## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

import os

PEDSIM_AGENTS = "pedsim_agents"

def pedsim_forces(prefix):
    PEDSIM_FORCES = f"${prefix}.pedsim_forces"

    packages = [PEDSIM_FORCES]
    
    # for pkg in next(os.walk(os.path.join("src","pedsim_agents","pedsim_forces","forcemodels")))[1]:
    #     packages.append(f"{PEDSIM_FORCES}.forcemodels.{pkg}") 

    return packages

def pedsim_semantic(prefix):
    PEDSIM_SEMANTIC = f"${prefix}.pedsim_semantic"

    packages = [PEDSIM_SEMANTIC]

    return packages

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        PEDSIM_AGENTS,
        *pedsim_forces(PEDSIM_AGENTS),
        *pedsim_semantic(PEDSIM_AGENTS)
    ],
    package_dir={'': 'src',}
)

setup(**setup_args)