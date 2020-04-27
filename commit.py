import sys, os

commit_message = input('Git message: ')
os.system('git add .')
os.system(f'git commit -m "{commit_message}"')
os.system('git push')
os.system('pip uninstall ramps_controller -y')
os.system('pip install git+https://github.com/Ladvien/ramps_controller.git')