## How to use it 

### Create object of **env** class

```
simpleEnv = env(modelpath = 'model.xml')
```

**env** class also takes other paramters which are helpful for running it on the server 

### **env** has *step* function which tries to grasp at the location provided in image space. It takes list of action as input as input.  

```
obs, reward, done, info = simpleEnv(action)
```

*action = [pixel_x, pixel_y, theta]*

*pixel_x & pixel_y* defines the location in image space where the grasp should be attempted

*pixel_x* :  range *(-1,1)*

*pixel_y* : range *(-1,1)*

*theta* defines the yaw of the gripper

*theta* : range *(-pi/2,pi/2)*

*obs* : provides image of the scene after taking aciton

*reward* : 1 if it grasp the object, 0 otherwise

*done* : done condition has not been implementd yet

*info* : None 

### *env* class also has reset function which will reset the environment to intial simulating condition

```
initObs = simpleEnv.reset()
```

*initObs* : intial image of scene
