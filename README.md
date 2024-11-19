# ReKep Replication
This is a personal replicate ReKep repo from [Official ReKep](https://github.com/huangwl18/ReKep). [Original Readme File](./ORIGIN_README.md)

## Branches
- main: the default demo used with a Fetch Robot 
- frankaSim: the official demo used with a FrankaPanda Robot. Franka Panda Robot end-effect frame differs from Fetch's, it's a bit 
- deployment: the real-world deployment with a Franka Panda Env

## TODO List
- [x] modify the control action space for franka padna
- [x] modify the desired grasp pose for franka panda
- [ ] the gripper action failed because of the mode of `MultiFingerGripperController` in FrankaPanda `gripper_0`