import sys, argparse, os, glob
sys.path.insert(1,'../classes')
from randomize_environment import get_spawn_pos

parser = argparse.ArgumentParser(description='Build initial positions for all environments')
parser.add_argument('-folder', type=str, help="(str) Folder with envs", default="../../../conf/environments/")
parser.add_argument('-n_agents',type=int,help="(int) number of agents",default = 10)
args = parser.parse_args()

for file in glob.glob("../../../conf/environments/rand_env_*"):
    env_name = file.split('/')[-1]
    os.system('cd ../../../ && python3 scripts/python/tools/complete_folder.py -env_name='+ env_name)


get_spawn_pos(args.n_agents,args.folder)
