#!/usr/bin/env python
import nest
stoptime = 200

proxy = nest.Create('music_cont_out_proxy', 1)
nest.SetStatus(proxy, {'port_name': 'out'})
nest.SetStatus(proxy, {'record_from': ["V_m"]})

neuron_grp = nest.Create('iaf_neuron', 2)
for i in range(len(neuron_grp)):
    # Funktioniert nicht, da hier connect 'anders herum' ist als fur spikes, d.h. receptor wird bei iaf_neuron gesetzt, nicht beim proxy
    nest.Connect(proxy, [neuron_grp[i]], 'one_to_one')


nest.Simulate(200)
print( nest.GetStatus(neuron_grp) )


