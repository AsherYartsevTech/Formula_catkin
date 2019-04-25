#!/usr/bin/env python


def callback_pp_controller(msg, sim):
    car_state = sim.client.getCarState()
    print('callback')
    print('gas: {}'.format(msg.gas))
    print('steer: {}'.format(msg.steering))

    car_state.throttle = msg.gas
    car_state.steering = msg.steering
    sim.client.setCarControls(car_state)
    return

