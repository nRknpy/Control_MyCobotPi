import torch
import torch.nn as nn
import torch.nn.functional as F


class MTRNNCell(nn.Module):
    def __init__(self,
                 input_dim,
                 io_hidden_dim,
                 fast_hidden_dim,
                 slow_hidden_dim,
                 io_tau=2,
                 fast_tau=5,
                 slow_tau=70):
        super(MTRNNCell, self).__init__()

        self.input_dim = input_dim

        self.io_hidden_dim = io_hidden_dim
        self.fast_hidden_dim = fast_hidden_dim
        self.slow_hidden_dim = slow_hidden_dim
        self.io_tau = io_tau
        self.fast_tau = fast_tau
        self.slow_tau = slow_tau

        self.input2io = nn.Linear(input_dim, io_hidden_dim)

        self.io2io = nn.Linear(io_hidden_dim, io_hidden_dim)
        self.io2fast = nn.Linear(io_hidden_dim, fast_hidden_dim)

        self.fast2io = nn.Linear(fast_hidden_dim, io_hidden_dim)
        self.fast2fast = nn.Linear(fast_hidden_dim, fast_hidden_dim)
        self.fast2slow = nn.Linear(fast_hidden_dim, slow_hidden_dim)

        self.slow2slow = nn.Linear(slow_hidden_dim, slow_hidden_dim)
        self.slow2fast = nn.Linear(slow_hidden_dim, fast_hidden_dim)

        self.activation = nn.Tanh()

    def forward(self, x, state=None):
        batch_size = x.shape[0]
        device = x.device

        if state is None:
            prev_io_hid, prev_fast_hid, prev_slow_hid = self.initial_hidden_states(
                batch_size, device)
        else:
            prev_io_hid, prev_fast_hid, prev_slow_hid = state

        io_hid = self.unit_forward(
            prev=prev_io_hid,
            new=[self.input2io(x),
                 self.io2io(prev_io_hid),
                 self.fast2io(prev_fast_hid)],
            tau=self.io_tau,
        )

        fast_hid = self.unit_forward(
            prev=prev_fast_hid,
            new=[self.io2fast(prev_io_hid),
                 self.fast2fast(prev_fast_hid),
                 self.slow2fast(prev_slow_hid)],
            tau=self.fast_tau,
        )

        slow_hid = self.unit_forward(
            prev=prev_slow_hid,
            new=[self.fast2slow(prev_fast_hid),
                 self.slow2slow(prev_slow_hid)],
            tau=self.slow_tau,
        )

        return self.activation(io_hid), self.activation(fast_hid), self.activation(slow_hid)

    def unit_forward(self, prev, new, tau):
        new_sum = torch.stack(new)
        new_sum = new_sum.sum(dim=0)
        return (1.0 - 1.0 / tau) * prev + 1.0 / tau * new_sum

    def initial_hidden_states(self, batch_size, device):
        io_hid = torch.zeros(batch_size, self.io_hidden_dim).to(device)
        fast_hid = torch.zeros(batch_size, self.fast_hidden_dim).to(device)
        slow_hid = torch.zeros(batch_size, self.slow_hidden_dim).to(device)
        return io_hid, fast_hid, slow_hid


class MTRNNPBCell(nn.Module):
    def __init__(self,
                 input_dim,
                 io_hidden_dim,
                 fast_hidden_dim,
                 slow_hidden_dim,
                 io_tau=2,
                 fast_tau=5,
                 slow_tau=70,
                 io_pb_dim=None,
                 fast_pb_dim=None,
                 slow_pb_dim=4):
        super(MTRNNPBCell, self).__init__()

        self.input_dim = input_dim

        self.io_hidden_dim = io_hidden_dim
        self.fast_hidden_dim = fast_hidden_dim
        self.slow_hidden_dim = slow_hidden_dim
        self.io_tau = io_tau
        self.fast_tau = fast_tau
        self.slow_tau = slow_tau

        self.io_pb_dim = io_pb_dim if io_pb_dim is not None else 0
        self.fast_pb_dim = fast_pb_dim if fast_pb_dim is not None else 0
        self.slow_pb_dim = slow_pb_dim if slow_pb_dim is not None else 0

        self.input2io = nn.Linear(input_dim + self.io_pb_dim, io_hidden_dim)

        self.io2io = nn.Linear(io_hidden_dim, io_hidden_dim)
        self.io2fast = nn.Linear(
            io_hidden_dim + self.fast_pb_dim, fast_hidden_dim)

        self.fast2io = nn.Linear(fast_hidden_dim, io_hidden_dim)
        self.fast2fast = nn.Linear(fast_hidden_dim, fast_hidden_dim)
        self.fast2slow = nn.Linear(
            fast_hidden_dim + self.slow_pb_dim, slow_hidden_dim)

        self.slow2slow = nn.Linear(slow_hidden_dim, slow_hidden_dim)
        self.slow2fast = nn.Linear(slow_hidden_dim, fast_hidden_dim)

        if self.io_pb_dim:
            self.io_pb = nn.Parameter(torch.zeros(io_pb_dim))
        else:
            self.io_pb = None
        if self.fast_pb_dim:
            self.fast_pb = nn.Parameter(torch.zeros(fast_pb_dim))
        else:
            self.fast_pb = None
        if self.slow_pb_dim:
            self.slow_pb = nn.Parameter(torch.zeros(slow_pb_dim))
        else:
            self.slow_pb = None

        self.activation = nn.Tanh()

    def forward(self, x, state=None):
        batch_size = x.shape[0]
        device = x.device

        if state is None:
            prev_io_hid, prev_fast_hid, prev_slow_hid = self.initial_hidden_states(
                batch_size, device)
        else:
            prev_io_hid, prev_fast_hid, prev_slow_hid = state

        io_hid = self.unit_forward(
            prev=prev_io_hid,
            new=[self.input2io(x),
                 self.io2io(prev_io_hid),
                 self.fast2io(prev_fast_hid)],
            tau=self.io_tau,
        )

        fast_hid = self.unit_forward(
            prev=prev_fast_hid,
            new=[self.io2fast(prev_io_hid),
                 self.fast2fast(prev_fast_hid),
                 self.slow2fast(prev_slow_hid)],
            tau=self.fast_tau,
        )

        slow_hid = self.unit_forward(
            prev=prev_slow_hid,
            new=[self.fast2slow(prev_fast_hid),
                 self.slow2slow(prev_slow_hid)],
            tau=self.slow_tau,
        )

        return self.activation(io_hid), self.activation(fast_hid), self.activation(slow_hid)

    def unit_forward(self, prev, new, tau, pb=None):
        new_sum = torch.stack(new)
        new_sum = new_sum.sum(dim=0)
        if pb is not None:
            new_sum = torch.concat([new_sum, pb], -1)
        return (1.0 - 1.0 / tau) * prev + 1.0 / tau * new_sum

    def initial_hidden_states(self, batch_size, device):
        io_hid = torch.zeros(batch_size, self.io_hidden_dim).to(device)
        fast_hid = torch.zeros(batch_size, self.fast_hidden_dim).to(device)
        slow_hid = torch.zeros(batch_size, self.slow_hidden_dim).to(device)
        return io_hid, fast_hid, slow_hid
