import pybullet as pb

RIGHT_REST = [0.4,
            -0.49826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.00396583422025,
            1.1980939705504309]

c = pb.connect(pb.GUI)

robot = pb.loadURDF("assets/franka_arm/panda_gripper.urdf", basePosition=[0.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071068, 0.7071068], useFixedBase=True)

for i in range(len(RIGHT_REST)):
    pb.resetJointState(robot, i, RIGHT_REST[i])

input("Press Enter to quit...")
pb.resetBasePositionAndOrientation(robot, [-0.86251631, -0.00165063, -0.0046883], [0, 0, 0.7071068, 0.7071068])
input("Press Enter to quit...")