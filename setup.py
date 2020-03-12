from distutils.core import setup

# Version meaning (X.Y.Z)
# X: Major version (e.g. vastly different scene, platform, etc)
# Y: Minor version (e.g. new tasks, major changes to existing tasks, etc)
# Z: Patch version (e.g. small changes to tasks, bug fixes, etc)

setup(name='rlbench',
      version='1.0.6',
      description='RLBench',
      author='Stephen James',
      author_email='slj12@ic.ac.uk',
      url='https://www.doc.ic.ac.uk/~slj12',
      packages=[
            'rlbench',
            'rlbench.backend',
            'rlbench.tasks',
            'rlbench.task_ttms',
            'rlbench.robot_ttms',
            'rlbench.sim2real',
            'rlbench.assets',
            'rlbench.gym'
      ],
      package_data={'': ['*.ttm', '*.obj', '**/**/*.ttm', '**/**/*.obj'],
                    'rlbench': ['task_design.ttt']},
      )
