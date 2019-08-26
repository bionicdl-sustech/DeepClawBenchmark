#!/bin/bash
rosservice call /bcap_service '{func_id: 3, vntArgs: [{vt: 8, value: "b-CAP"}, {vt: 8, value: "CaoProv.DENSO.VRC"}, {vt: 8, value: "localhost"}, {vt: 8, value: ""}] }'
rosservice call /bcap_service '{func_id: 17, vntArgs: [{vt: 19, value: "152"}, {vt: 8, value: "HandMoveA"}, {vt: 8195, value: "29.5, 100"}] }'
