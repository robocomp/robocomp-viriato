# HumanSceneSim
This component simulate the position of several persons and change their pose in the RCIS as well as in AGM.

## Dataset
The SALSA dateset is used here.\
You can find the dataset [here..](http://tev.fbk.eu/salsa)
## Configuration parameters
As any other component, *HumanSceneSim* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```
## Compiling the component
```
cd <HumanSceneSim path>
mkdir build
cd build
cmake ..
make
```


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <HumanSceneSim's path>
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
./bin/HumanSceneSim config
```
