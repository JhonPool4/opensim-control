# Package information
This package use Opensim 4.3 to simulate the dynamics of an arm with 2 Dof and 6 muscles.

# Requirements
- Python 3.8
- Opensim 4.3 
# Installation
## 1. clone the repository
<pre><code>git clone https://github.com/JhonPool4/opensim-control.git </code></pre> 

## 2. create a conda enviornment
<pre><code>conda env create -f conda_config/custom_env.yml </code></pre> 
activate conda environemnt
<pre><code>conda activate opensim-py38 </code></pre>

## 3. configuration of Opensim 4.3
First intall Opensim from the offitial page [link](https://simtk.org/frs/?group_id=91)

Second, you need to indicate the location of the opensim libraries. For this, we will add paths to the environment variables (PATH and PYTHONPATH)

###  For Windows users
Go to ~/Opensim 4.3/sdk/Python
<pre><code>python setup_win_python38.py</code></pre> 
<pre><code>python -m pip install .</code></pre> 
Add path of the dynamic libraries
<pre><code>Path: C:\OpenSim 4.3\bin</code></pre> 
Add python libraries
<pre><code>PYTHONPATH: C:\OpenSim 4.3\sdk\Python</code></pre> 


### For MAC users
Go to ~/Opensim 4.3/sdk
<pre><code>python -m pip install .</code></pre> 
Open your .bash_profile and add the dynamic libraries
<pre><code>export DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:/Applications/OpenSim 4.3/OpenSim 4.3.app/Contents/Resources/opensim/sdk/lib"</code></pre> 
<pre><code>export DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:/Applications/OpenSim 4.3/OpenSim 4.3.app/Contents/Resources/opensim/sdk/Simbody/lib"</code></pre> 


# Test 
After activate conda environment
<pre><code>python test_arm_control.py</code></pre> 

