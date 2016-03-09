#!/usr/bin/env python
import nest

proxy = nest.Create('music_cont_out_proxy', 1)
nest.SetStatus(proxy, {'port_name': 'out'})
nest.SetStatus(proxy, {'record_from': ["V_m"], 'max_buffered': 1, 'interval': 0.1})
nest.SetStatus(proxy, {'index_map': [0, 1]})

neuron_grp = nest.Create('iaf_cond_exp', 2)
for i in range(len(neuron_grp)):
    # Funktioniert nicht, da hier connect 'anders herum' ist als fur spikes, d.h. receptor wird bei iaf_neuron gesetzt, nicht beim proxy
    nest.Connect(proxy, [neuron_grp[i]])
nest.SetStatus([neuron_grp[0]], "I_e", 300.)
nest.SetStatus([neuron_grp[1]], "I_e", 600.)



nest.Simulate(6000)
print( nest.GetStatus(neuron_grp) )


