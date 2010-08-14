## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):    
    obj = bld.create_ns3_program('vanet-highway-test')
    obj.source = [
	'vanet-highway-test.cc',
	'Highway.cc',
	'Vehicle.cc',
	'Obstacle.cc',
	'Model.cc',
	'LaneChange.cc'
	]



