#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# nest_script.py
#
# This file is part of NEST.
#
# Copyright (C) 2004 The NEST Initiative
#
# NEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# NEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with NEST.  If not, see <http://www.gnu.org/licenses/>.

import music
import nest
import numpy
import pyNN.nest as sim

sim.setup(verbosity="WARNING")
proxy = nest.Create('music_cont_out_proxy', 1)
neuron_grp = nest.Create('iaf_neuron', 2)

print neuron_grp
nest.SetStatus(proxy, {'record_from': ["V_m"]})
nest.SetStatus(proxy, {'port_name': 'out', 'target_gids': neuron_grp})

nest.SetStatus([neuron_grp[0]], "I_e", 400.)
nest.SetStatus([neuron_grp[1]], "I_e", 800.)

while True:
    nest.Simulate(20.0)
