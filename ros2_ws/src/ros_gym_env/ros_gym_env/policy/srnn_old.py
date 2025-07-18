import torch.nn as nn
import torch
import numpy as np


def init(module, weight_init, bias_init, gain=1):
    weight_init(module.weight.data, gain=gain)
    bias_init(module.bias.data)
    return module


class SRNN_OLD(nn.Module):
    """
    Class representing the SRNN model
    """
    def __init__(self, obs_space_dict, config):
        """
        Initializer function
        params:
        config : Training arguments
        infer : Training or test time (True at test time)
        """
        super(SRNN_OLD, self).__init__()
        self.is_recurrent = True
        self.config=config

        self.human_num = obs_space_dict['spatial_edges'].shape[0]

        self.seq_length = config.ppo.num_steps
        self.nenv = config.training.num_processes
        self.nminibatch = config.ppo.num_mini_batch

        # Store required sizes
        self.human_node_rnn_size = config.SRNN.human_node_rnn_size
        self.human_human_edge_rnn_size =  config.SRNN.human_human_edge_rnn_size
        self.output_size = config.SRNN.human_node_output_size

        # Initialize the Node and Edge RNNs
        self.humanNodeRNN = EndRNNLidar(config)

        # Initialize robot-human attention module
        self.attn = EdgeAttention_M(config)

        self.init_ = lambda m: init(m, nn.init.orthogonal_, lambda x: nn.init.
                               constant_(x, 0), np.sqrt(2))

        hidden_size = self.output_size

        self.actor = nn.Sequential(
            self.init_(nn.Linear(self.config.SRNN.human_node_rnn_size, hidden_size)), nn.Tanh(),
            self.init_(nn.Linear(hidden_size, hidden_size)), nn.Tanh())

        self.critic = nn.Sequential(
            self.init_(nn.Linear(self.config.SRNN.human_node_rnn_size, hidden_size)), nn.Tanh(),
            self.init_(nn.Linear(hidden_size, hidden_size)), nn.Tanh())


        self.critic_linear = self.init_(nn.Linear(hidden_size, 1))
        robot_size = obs_space_dict['robot_node'].shape[1]

        self.robot_linear = nn.Sequential(self.init_(nn.Linear(robot_size, config.SRNN.robot_embedding_size)), nn.ReLU())
        # self.human_node_final_linear=self.init_(nn.Linear(self.output_size,2))

        self.spatial_attn = SpatialEdgeSelfAttn(config)
        self.spatial_linear = nn.Sequential(self.init_(nn.Linear(self.config.SRNN.self_attn_size, config.SRNN.human_embedding_size)), nn.ReLU())
        # self.spatial_linear = nn.Sequential(self.init_(nn.Linear(self.config.SRNN.self_attn_size, 64)), nn.ReLU())


        # lookup table: given a lidar angular resolution, what is the output size of lidar CNN after flattening
        # 608 if angular resolution is 1, 256 if angular resolution of lidar is 2
        self.lidar_conv_out_size_lookup = {1: 608, 2: 256, 4: 64}

        self.lidar_input_size = int(360. / self.config.lidar.angular_res)
        self.lidar_embed_size = config.SRNN.obs_embedding_size
        self.lidar_channel_num = 1
        self.lidar_embed_conv_out_size = self.lidar_conv_out_size_lookup[self.config.lidar.angular_res]

        # Linear layers to embed inputs
        # 1d conv
        self.lidar_embed = nn.Sequential(self.init_(nn.Conv1d(self.lidar_channel_num, 16, 10, stride=2)), nn.ReLU(),
                                         # (1, 360) -> (32, 176)
                                         self.init_(nn.Conv1d(16, 32, 5, stride=2)), nn.ReLU(),  # (32, 176) -> (32, 86)
                                         self.init_(nn.Conv1d(32, 32, 5, stride=2)), nn.ReLU(),  # (32, 86) -> (32, 41)
                                         self.init_(nn.Conv1d(32, 32, 5, stride=2)), nn.ReLU(),  # (32, 41) -> (32, 19)
                                         Flatten(),
                                         self.init_(nn.Linear(self.lidar_embed_conv_out_size, self.lidar_embed_size)),
                                         nn.ReLU(),
                                         )



    def forward(self, inputs, rnn_hxs, masks, infer=False):
        if infer:
            # Test time
            seq_length = 1
            nenv = self.nenv

        else:
            # Training time
            seq_length = self.seq_length
            nenv = self.nenv // self.nminibatch

        robot_states = reshapeT(inputs['robot_node'], seq_length, nenv)
        spatial_edges = reshapeT(inputs['spatial_edges'], seq_length, nenv)
        detected_human_num = inputs['detected_human_num'].squeeze(-1).cpu().int()
        # [seq len, batch size, 2, pc num] -> [seq_len*batch_size,2, pc num]
        lidar_in = inputs['point_clouds'].reshape(seq_length * nenv, self.lidar_channel_num, self.lidar_input_size)

        hidden_states_node_RNNs = reshapeT(rnn_hxs['rnn'], 1, nenv)

        masks = reshapeT(masks, seq_length, nenv)

        # embed robot states
        robot_states = self.robot_linear(robot_states)

        # embed lidar pc
        lidar_features = self.lidar_embed(lidar_in)
        # reshape it back to dim=4
        lidar_features = lidar_features.view(seq_length, nenv, 1, self.lidar_embed_size)

        # embed human states, add various attention weights to human embeddings
        # human-human self attention
        spatial_attn_out=self.spatial_attn(spatial_edges, detected_human_num).view(seq_length, nenv, self.human_num, -1)
        # [seq len, nenv, human num, 64] (64 is human_embedding_size)
        output_spatial = self.spatial_linear(spatial_attn_out)  # (seq len, nenv, human num, 64)

        # robot-human attention
        hidden_attn_weighted, _ = self.attn(robot_states, output_spatial, detected_human_num)

        # Do a forward pass through nodeRNN
        outputs, h_nodes \
            = self.humanNodeRNN(robot_states, hidden_attn_weighted, lidar_features, hidden_states_node_RNNs, masks)

        # Update the hidden and cell states
        all_hidden_states_node_RNNs = h_nodes
        outputs_return = outputs

        rnn_hxs['rnn'] = all_hidden_states_node_RNNs

        # x is the output of the robot node and will be sent to actor and critic
        x = outputs_return[:, :, 0, :]

        hidden_critic = self.critic(x)
        hidden_actor = self.actor(x)

        for key in rnn_hxs:
            rnn_hxs[key] = rnn_hxs[key].squeeze(0)

        if infer:
            return self.critic_linear(hidden_critic).squeeze(0), hidden_actor.squeeze(0), rnn_hxs
        else:
            return self.critic_linear(hidden_critic).view(-1, 1), hidden_actor.view(-1, self.output_size), rnn_hxs
        



class SpatialEdgeSelfAttn(nn.Module):
    def __init__(self, config):
        super(SpatialEdgeSelfAttn, self).__init__()
        self.config = config

        if config.robot.policy == 'selfAttn_merge_srnn_lidar_human_pc':
            self.input_size = config.SRNN.human_embedding_size # 48
        elif config.robot.policy == 'homo_transformer_obs':
            self.input_size = config.SRNN.robot_embedding_size
        else:
            # Store required sizes
            if self.config.env.env_name in ['CrowdSimVarNum-v0']:
                self.input_size = 2
            elif self.config.env.env_name in ['CrowdSim3DTB-v0', 'CrowdSim3DTbObs-v0', 'CrowdSim3DTbObsHie-v0','rosTurtlebot2iEnv-v0']:
                if self.config.ob_space.add_human_vel:
                    self.input_size = 4
                else:
                    self.input_size = 2
            else:
                raise ValueError("Unknown environment name")

        self.num_attn_heads=8
        self.attn_size=self.config.SRNN.self_attn_size


        # Linear layer to embed input
        self.embedding_layer = nn.Sequential(nn.Linear(self.input_size, self.attn_size), nn.ReLU()
                                             )

        self.q_linear = nn.Linear(self.attn_size, self.attn_size)
        self.v_linear = nn.Linear(self.attn_size, self.attn_size)
        self.k_linear = nn.Linear(self.attn_size, self.attn_size)

        # multi-head self attention
        self.multihead_attn=torch.nn.MultiheadAttention(self.attn_size, self.num_attn_heads)


    # Given a list of sequence lengths, create a mask to indicate which indices are padded
    # e.x. Input: [3, 1, 4], max_human_num = 5
    # Output: [[1, 1, 1, 0, 0], [1, 0, 0, 0, 0], [1, 1, 1, 1, 0]]
    def create_attn_mask(self, each_seq_len, seq_len, nenv, max_human_num):
        # mask with value of False means padding and should be ignored by attention
        # why +1: use a sentinel in the end to handle the case when each_seq_len = 18
        if not self.config.training.cuda:
            mask = torch.zeros(seq_len * nenv, max_human_num + 1).cpu()
        else:
            mask = torch.zeros(seq_len*nenv, max_human_num+1).cuda()
        mask[torch.arange(seq_len*nenv), each_seq_len.long()] = 1.
        mask = torch.logical_not(mask.cumsum(dim=1))
        # remove the sentinel
        mask = mask[:, :-1].unsqueeze(-2) # seq_len*nenv, 1, max_human_num
        return mask


    def forward(self, inp, each_seq_len):
        '''
        Forward pass for the model
        params:
        inp : input edge features
        each_seq_len: the true length of the sequence. Should be the number of detected humans
        '''
        # inp is padded sequence [seq_len, nenv, max_human_num, 2]
        seq_len, nenv, max_human_num, _ = inp.size()
        attn_mask = self.create_attn_mask(each_seq_len, seq_len, nenv, max_human_num)  # [seq_len*nenv, 1, max_human_num]
        attn_mask=attn_mask.squeeze(1) # if we use pytorch builtin function
        input_emb=self.embedding_layer(inp).view(seq_len*nenv, max_human_num, -1) # [seq_len*nenv, max_human_num, self.attn_size]
        input_emb=torch.transpose(input_emb, dim0=0, dim1=1) # [max_human_num, seq_len*nenv, self.attn_size]
        q=self.q_linear(input_emb)
        k=self.k_linear(input_emb)
        v=self.v_linear(input_emb)

        # [max_human_num, seq_len*nenv, self.attn_size]
        z,_=self.multihead_attn(q, k, v, key_padding_mask=torch.logical_not(attn_mask)) # if we use pytorch builtin function
        z=torch.transpose(z, dim0=0, dim1=1) # [seq_len*nenv, max_human_num, self.attn_size]
        return z


class EdgeAttention_M(nn.Module):
    '''
    Class representing the attention module
    attn_type: RH means robot-human attention, RO means robot-obstacle attention
    '''
    def __init__(self, config):
        '''
        Initializer function
        params:
        config : Training arguments
        infer : Training or test time (True at test time)
        '''
        super(EdgeAttention_M, self).__init__()

        self.config = config

        # Store required sizes
        self.human_embedding_size = config.SRNN.human_embedding_size

        self.num_attention_head = config.SRNN.hr_attn_head_num
        self.attention_size = config.SRNN.hr_attention_size

        # Linear layer to embed temporal edgeRNN hidden state
        self.temporal_edge_layer=nn.ModuleList()
        self.spatial_edge_layer=nn.ModuleList()

        for _ in range(self.num_attention_head):
            self.temporal_edge_layer.append(nn.Linear(self.human_embedding_size, self.attention_size))
            # Linear layer to embed spatial edgeRNN hidden states
            self.spatial_edge_layer.append(nn.Linear(self.human_embedding_size, self.attention_size))

        if self.num_attention_head > 1:
            self.final_attn_linear = nn.Linear(self.human_embedding_size * self.num_attention_head, self.human_embedding_size)

    def create_attn_mask(self, each_seq_len, seq_len, nenv, max_human_num):
        # mask with value of False means padding and should be ignored by attention
        # why +1: use a sentinel in the end to handle the case when each_seq_len = 18
        if not self.config.training.cuda:
            mask = torch.zeros(seq_len * nenv, max_human_num + 1).cpu()
        else:
            mask = torch.zeros(seq_len * nenv, max_human_num + 1).cuda()
        mask[torch.arange(seq_len * nenv), each_seq_len.long()] = 1.
        mask = torch.logical_not(mask.cumsum(dim=1))
        # remove the sentinel
        mask = mask[:, :-1].unsqueeze(-2)  # seq_len*nenv, 1, max_human_num
        return mask

    def att_func(self, temporal_embed, spatial_embed, h_spatials, attn_mask=None):
        seq_len, nenv, num_edges, h_size = h_spatials.size()  # [1, 12, 30, 256] in testing,  [12, 30, 256] in training
        attn = temporal_embed * spatial_embed
        attn = torch.sum(attn, dim=3)

        # Variable length
        temperature = num_edges / np.sqrt(self.attention_size)
        attn = torch.mul(attn, temperature)

        if attn_mask is not None:
            attn = attn.masked_fill(attn_mask == 0, -1e9)

        # Softmax
        # [seq len, nenv, human num]
        attn = torch.nn.functional.softmax(attn, dim=-1)
        # print(np.round(attn[0, 0, 0].cpu().numpy(), 2))

        # reshape h_spatials and attn
        # [seq_len*nenv, human num, attention size] -> [seq_len*nenv, attention size, human num]
        h_spatials = h_spatials.view(seq_len * nenv, self.human_num, h_size).permute(0, 2, 1)

        # add all weighted human embeddings together, size of weight_value is [batch, attention size, 1]
        attn = attn.view(seq_len * nenv, self.human_num).unsqueeze(-1)  # [seq_len*nenv, human num, 1]
        weighted_value = torch.bmm(h_spatials, attn)  # [seq_len*nenv*6, 256, 1]

        # reshape back
        weighted_value = weighted_value.squeeze(-1).view(seq_len, nenv, 1, h_size)  # [seq_len, 12, 6 or 1, 256]

        return weighted_value, attn


    # h_temporal: [seq_len, nenv, 1, 256]
    # h_spatials: [seq_len, nenv, 5, 256]
    def forward(self, h_temporal, h_spatials, each_seq_len):
        '''
        Forward pass for the model
        params:
        h_temporal : Hidden state of the temporal edgeRNN
        h_spatials : Hidden states of all spatial edgeRNNs connected to the node.
        '''
        seq_len, nenv, max_human_num, _ = h_spatials.size()
        # find the number of humans by the size of spatial edgeRNN hidden state
        self.human_num = max_human_num

        weighted_value_list, attn_list=[],[]
        for i in range(self.num_attention_head):

            # Embed the temporal edgeRNN hidden state
            temporal_embed = self.temporal_edge_layer[i](h_temporal)
            # temporal_embed = temporal_embed.squeeze(0)

            # Embed the spatial edgeRNN hidden states
            spatial_embed = self.spatial_edge_layer[i](h_spatials)

            # Dot based attention
            temporal_embed = temporal_embed.repeat_interleave(self.human_num, dim=2)

            attn_mask = self.create_attn_mask(each_seq_len, seq_len, nenv, max_human_num)  # [seq_len*nenv, 1, max_human_num]
            attn_mask = attn_mask.squeeze(-2).view(seq_len, nenv, max_human_num)
            weighted_value,attn=self.att_func(temporal_embed, spatial_embed, h_spatials, attn_mask=attn_mask)
            weighted_value_list.append(weighted_value)
            attn_list.append(attn)

        if self.num_attention_head > 1:
            return self.final_attn_linear(torch.cat(weighted_value_list, dim=-1)), attn_list
        else:
            return weighted_value_list[0], attn_list[0]



















class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)

def reshapeT(T, seq_length, nenv):
    shape = T.size()[1:]
    return T.unsqueeze(0).reshape((seq_length, nenv, *shape))


class RNNBase(nn.Module):
    # edge: True -> edge RNN, False -> node RNN
    def __init__(self, config, edge):
        super(RNNBase, self).__init__()
        self.config = config

        # if this is an edge RNN
        if edge:
            self.gru = nn.GRU(config.SRNN.human_human_edge_embedding_size, config.SRNN.human_human_edge_rnn_size)
        # if this is a node RNN
        else:
            self.gru = nn.GRU(config.SRNN.human_node_embedding_size*2, config.SRNN.human_node_rnn_size)

        for name, param in self.gru.named_parameters():
            if 'bias' in name:
                nn.init.constant_(param, 0)
            elif 'weight' in name:
                nn.init.orthogonal_(param)

    # x: [seq_len, nenv, 6 or 30 or 36, ?]
    # hxs: [1, nenv, 6 or 30 or 36, ?]
    # masks: [1, nenv, 1]
    def _forward_gru(self, x, hxs, masks):
        # for acting model, input shape[0] == hidden state shape[0]
        if x.size(0) == hxs.size(0):
            # use env dimension as batch
            # [1, 12, 6, ?] -> [1, 12*6, ?] or [30, 6, 6, ?] -> [30, 6*6, ?]
            seq_len, nenv, agent_num, _ = x.size()
            x = x.view(seq_len, nenv*agent_num, -1)
            hxs_times_masks = hxs * (masks.view(seq_len, nenv, 1, 1))
            hxs_times_masks = hxs_times_masks.view(seq_len, nenv*agent_num, -1)
            x, hxs = self.gru(x, hxs_times_masks) # we already unsqueezed the inputs in SRNN forward function
            x = x.view(seq_len, nenv, agent_num, -1)
            hxs = hxs.view(seq_len, nenv, agent_num, -1)

        # during update, input shape[0] * nsteps (30) = hidden state shape[0]
        else:

            # N: nenv, T: seq_len, agent_num: node num or edge num
            T, N, agent_num, _ = x.size()
            # x = x.view(T, N, agent_num, x.size(2))

            # Same deal with masks
            masks = masks.view(T, N)

            # Let's figure out which steps in the sequence have a zero for any agent
            # We will always assume t=0 has a zero in it as that makes the logic cleaner
            # for the [29, num_env] boolean array, if any entry in the second axis (num_env) is True -> True
            # to make it [29, 1], then select the indices of True entries
            has_zeros = ((masks[1:] == 0.0) \
                            .any(dim=-1)
                            .nonzero()
                            .squeeze()
                            .cpu())

            # +1 to correct the masks[1:]
            if has_zeros.dim() == 0:
                # Deal with scalar
                has_zeros = [has_zeros.item() + 1]
            else:
                has_zeros = (has_zeros + 1).numpy().tolist()

            # add t=0 and t=T to the list
            has_zeros = [0] + has_zeros + [T]

            # hxs = hxs.unsqueeze(0)
            # hxs = hxs.view(hxs.size(0), hxs.size(1)*hxs.size(2), hxs.size(3))
            outputs = []
            for i in range(len(has_zeros) - 1):
                # We can now process steps that don't have any zeros in masks together!
                # This is much faster
                start_idx = has_zeros[i]
                end_idx = has_zeros[i + 1]

                # x and hxs have 4 dimensions, merge the 2nd and 3rd dimension
                x_in = x[start_idx:end_idx]
                x_in = x_in.view(x_in.size(0), x_in.size(1)*x_in.size(2), x_in.size(3))
                hxs = hxs.view(hxs.size(0), N, agent_num, -1)
                hxs = hxs * (masks[start_idx].view(1, -1, 1, 1))
                hxs = hxs.view(hxs.size(0), hxs.size(1) * hxs.size(2), hxs.size(3))
                rnn_scores, hxs = self.gru(x_in, hxs)

                outputs.append(rnn_scores)

            # assert len(outputs) == T
            # x is a (T, N, -1) tensor
            x = torch.cat(outputs, dim=0)
            # flatten
            x = x.view(T, N, agent_num, -1)
            hxs = hxs.view(1, N, agent_num, -1)

        return x, hxs


class EndRNNLidar(RNNBase):
    '''
    Class representing human Node RNNs in the st-graph
    '''
    def __init__(self, config):
        '''
        Initializer function
        params:
        config : Training arguments
        infer : Training or test time (True at test time)
        '''
        super(EndRNNLidar, self).__init__(config, edge=False)

        self.config = config

        # Store required sizes
        self.rnn_size = config.SRNN.human_node_rnn_size
        self.output_size = config.SRNN.human_node_output_size
        self.embedding_size = config.SRNN.human_node_embedding_size
        self.edge_rnn_size = config.SRNN.human_human_edge_rnn_size

        # Linear layer to embed robot state (256, 64)
        self.robot_linear = nn.Linear(config.SRNN.robot_embedding_size, self.embedding_size//2)

        # linear layer to embed lidar scan features
        self.lidar_linear = nn.Linear(config.SRNN.obs_embedding_size, self.embedding_size)

        # ReLU and Dropout layers
        self.relu = nn.ReLU()

        # Linear layer to embed attention module output (256, 64)
        self.human_linear = nn.Linear(config.SRNN.human_embedding_size, self.embedding_size//2)


    def forward(self, robot_s, h_spatial_other, lidar_features, h, masks):
        '''
        Forward pass for the model
        params:
        pos : input position
        h_temporal : hidden state of the temporal edgeRNN corresponding to this node
        h_spatial_other : output of the attention module
        h : hidden state of the current nodeRNN
        c : cell state of the current nodeRNN
        '''
        # Encode the input position
        encoded_input = self.relu(self.robot_linear(robot_s))

        h_edges_embedded = self.relu(self.human_linear(h_spatial_other))

        lidar_embedded = self.relu(self.lidar_linear(lidar_features))

        concat_encoded = torch.cat((encoded_input, h_edges_embedded, lidar_embedded), -1)

        x, h_new = self._forward_gru(concat_encoded, h, masks)

        return x, h_new