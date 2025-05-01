# policy-gradient-adaptive-control
Here is the code for policy gradient adaptive control algorithms (including data-enabled policy optimization (DeePO) as a special case)

Corresponding papers are: https://arxiv.org/abs/2503.02985, https://arxiv.org/abs/2401.14871

To use, first run data_generate.mlx to get offline data. Then, run direct.mlx for DeePO, for example. The sdp.m is used to solve the initial gain.
