import torch
import torch.nn as nn
import torch.nn.functional as F

from layer.MTRNN import MTRNNCell, MTRNNPBCell
from layer.image_processing import ImageEncoder, ImageDecoder, ImageEncoder256, ImageDecoder256


class AEMTRNN(nn.Module):
    def __init__(self,
                 rnn_hidden_dims,
                 image_feat_dim,
                 decoder_input_dim,
                 joint_dim,
                 rnn_io_tau=2,
                 rnn_fast_tau=5,
                 rnn_slow_tau=70,):
        super(AEMTRNN, self).__init__()

        self.rnn_input_dim = image_feat_dim + joint_dim
        self.rnn_io_hidden_dim, self.rnn_fast_hidden_dim, self.rnn_slow_hidden_dim = rnn_hidden_dims
        self.rnn_io_tau = rnn_io_tau
        self.rnn_fast_tau = rnn_fast_tau
        self.rnn_slow_tau = rnn_slow_tau

        self.decoder_input_dim = decoder_input_dim

        self.image_encoder = ImageEncoder256(image_feat_dim)
        self.rnn = MTRNNCell(self.rnn_input_dim,
                             self.rnn_io_hidden_dim,
                             self.rnn_fast_hidden_dim,
                             self.rnn_slow_hidden_dim,
                             self.rnn_io_tau,
                             self.rnn_fast_tau,
                             self.rnn_slow_tau)

        self.joint_decoder = nn.Sequential(nn.Linear(self.rnn_io_hidden_dim, joint_dim),
                                           nn.Tanh())
        self.fn_for_image_decoder = nn.Sequential(nn.Linear(self.rnn_io_hidden_dim + joint_dim, self.decoder_input_dim),
                                                  nn.ReLU(True))
        self.image_decoder = ImageDecoder256(self.decoder_input_dim)

    def forward(self, image, joint, rnn_state=None):
        image_feat = self.image_encoder(image)
        rnn_input = torch.concat([image_feat, joint], -1)

        rnn_hidden_states = self.rnn(rnn_input, rnn_state)

        next_joint = self.joint_decoder(rnn_hidden_states[0])

        decoder_input = self.fn_for_image_decoder(
            torch.concat([rnn_hidden_states[0], next_joint], -1))
        pred_next_image = self.image_decoder(decoder_input)

        return pred_next_image, next_joint, rnn_hidden_states


class AEMTRNNPB(nn.Module):
    def __init__(self,
                 rnn_hidden_dims,
                 image_feat_dim,
                 decoder_input_dim,
                 joint_dim,
                 rnn_io_tau=2,
                 rnn_fast_tau=5,
                 rnn_slow_tau=70,
                 io_pb_dim=None,
                 fast_pb_dim=None,
                 slow_pb_dim=None,):
        super(AEMTRNN, self).__init__()

        self.rnn_input_dim = image_feat_dim + joint_dim
        self.rnn_io_hidden_dim, self.rnn_fast_hidden_dim, self.rnn_slow_hidden_dim = rnn_hidden_dims
        self.rnn_io_tau = rnn_io_tau
        self.rnn_fast_tau = rnn_fast_tau
        self.rnn_slow_tau = rnn_slow_tau
        self.io_pb_dim = io_pb_dim
        self.fast_pb_dim = fast_pb_dim
        self.slow_pb_dim = slow_pb_dim

        self.decoder_input_dim = decoder_input_dim

        self.image_encoder = ImageEncoder(image_feat_dim)
        self.rnn = MTRNNPBCell(self.rnn_input_dim,
                               self.rnn_io_hidden_dim,
                               self.rnn_fast_hidden_dim,
                               self.rnn_slow_hidden_dim,
                               self.rnn_io_tau,
                               self.rnn_fast_tau,
                               self.rnn_slow_tau,
                               self.io_pb_dim,
                               self.fast_pb_dim,
                               self.slow_pb_dim)

        self.joint_decoder = nn.Sequential(nn.Linear(self.rnn_io_hidden_dim, joint_dim),
                                           nn.Tanh())
        self.fn_for_image_decoder = nn.Sequential(nn.Linear(self.rnn_io_hidden_dim + joint_dim, self.decoder_input_dim),
                                                  nn.ReLU(True))
        self.image_decoder = ImageDecoder(self.decoder_input_dim)

    def forward(self, image, joint, rnn_state=None):
        image_feat = self.image_encoder(image)
        rnn_input = torch.concat([image_feat, joint], -1)

        rnn_hidden_states = self.rnn(rnn_input, rnn_state)

        next_joint = self.joint_decoder(rnn_hidden_states[0])

        decoder_input = self.fn_for_image_decoder(
            torch.concat([rnn_hidden_states[0], next_joint], -1))
        pred_next_image = self.image_decoder(decoder_input)

        return pred_next_image, next_joint, rnn_hidden_states


class AEMTRNNSoftMoE(nn.Module):
    def __init__(self,
                 num_rnn=16,
                 rnn_hidden_dims=[64, 64, 64],
                 image_feat_dim=16,
                 joint_dim=8,
                 rnn_io_tau=2,
                 rnn_fast_tau=5,
                 rnn_slow_tau=70,):
        super(AEMTRNNSoftMoE, self).__init__()
        self.num_rnn = num_rnn

        self.rnn_input_dim = image_feat_dim + joint_dim
        self.rnn_io_hidden_dim, self.rnn_fast_hidden_dim, self.rnn_slow_hidden_dim = rnn_hidden_dims
        self.rnn_io_tau = rnn_io_tau
        self.rnn_fast_tau = rnn_fast_tau
        self.rnn_slow_tau = rnn_slow_tau

        self.decoder_input_dim = self.rnn_io_hidden_dim + joint_dim

        self.image_encoder = ImageEncoder(image_feat_dim)

        self.rnns = nn.ModuleList(
            [
                MTRNNCell(self.rnn_input_dim,
                          self.rnn_io_hidden_dim,
                          self.rnn_fast_hidden_dim,
                          self.rnn_slow_hidden_dim,
                          self.rnn_io_tau,
                          self.rnn_fast_tau,
                          self.rnn_slow_tau) for _ in range(self.num_rnn)
            ]
        )
        self.error_predictors = nn.ModuleList(
            [
                nn.Sequential(nn.Linear(self.rnn_slow_hidden_dim, 1),
                              nn.ReLU(True)) for _ in range(self.num_rnn)
            ]
        )

        self.joint_decoder = nn.Sequential(nn.Linear(self.rnn_io_hidden_dim, joint_dim),
                                           nn.ReLU(True))
        self.image_decoder = ImageDecoder(self.decoder_input_dim)

    def forward(self, image, joint, rnn_states=None):
        image_feat = self.image_encoder(image)
        rnn_input = torch.concat([image_feat, joint], -1)

        rnn_hidden_states = []
        pred_errors = []
        expert_joints = []
        expert_images = []
        for i in range(self.num_rnn):
            if rnn_states is None:
                state = None
            else:
                state = torch.permute(rnn_states[:, i], (1, 0, 2))
            state = self.rnns[i](rnn_input, state)
            state = torch.stack(state, dim=1)
            rnn_hidden_states.append(state)
            pred_errors.append(self.error_predictors[i](state[:, 2]))

            with torch.no_grad():
                pred_joint = self.joint_decoder(state[:, 0])
                pred_image = self.image_decoder(
                    torch.concat([state[:, 2], pred_joint], -1))
                expert_joints.append(pred_joint)
                expert_images.append(pred_image)

        expert_joints = torch.stack(expert_joints)
        expert_images = torch.stack(expert_images)

        rnn_hidden_states = torch.stack(rnn_hidden_states, dim=1)
        pred_errors = torch.concat(pred_errors, dim=1)
        weight = F.softmax(-pred_errors)

        hidden_states = torch.bmm(
            weight.unsqueeze(1), rnn_hidden_states[:, :, 2]).squeeze(1)

        next_joint = self.joint_decoder(hidden_states)

        decoder_input = torch.concat([hidden_states, next_joint], -1)
        pred_next_image = self.image_decoder(decoder_input)

        return pred_next_image, next_joint, rnn_hidden_states, weight, expert_joints, expert_images
