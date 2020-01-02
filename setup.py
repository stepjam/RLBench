from distutils.core import setup

setup(name='rlbench',
      version='1.0',
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
      package_data={'': ['*.ttm', '*.obj'],
                    'rlbench': ['task_design.ttt']},
      )
