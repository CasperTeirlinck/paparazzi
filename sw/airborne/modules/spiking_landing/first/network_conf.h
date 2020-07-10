//Auto-generated
#include "Network.h"
#include "connection_conf_inhid.h"
#include "connection_conf_hidout.h"
#include "neuron_conf_hid.h"
#include "neuron_conf_out.h"
float const centers[] = {-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
float const actions[] = {-0.800000011920929, -0.7315789461135864, -0.6631579399108887, -0.5947368741035461, -0.5263158082962036, -0.4578947424888611, -0.38947370648384094, -0.3210526704788208, -0.25263160467147827, -0.18421053886413574, -0.11578947305679321, -0.04736846685409546, 0.02105259895324707, 0.0894736647605896, 0.15789467096328735, 0.22631579637527466, 0.2947368025779724, 0.36315780878067017, 0.43157893419265747, 0.4999999403953552};
NetworkConf const conf = {2, 1, 1.0, actions, centers, 2, 4, 5, 20, &conf_inhid, &conf_hid, &conf_hidout, &conf_out};
