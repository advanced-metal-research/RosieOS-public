"""MuJoCo cell simulator for Robot V4.

It stands where RTCore and the nine A6-EC drives stand: it binds RTCore's UDP
port, takes the Motion Server's jog stream, forms a CSP target the way RTCore
does, runs the drives' position/speed cascade on a MuJoCo model of the cell,
and answers with `ServoFeedbackV4` read off the simulated encoders. Nothing
above it -- `robot-v4-cartesiand`, the OLP, the Deck -- knows the difference.
"""
