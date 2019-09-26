from distutils.core import setup
import setuptools


setup(name='rlbench',
      version='1.0',
      description='RLBench',
      author='Stephen James',
      author_email='slj12@ic.ac.uk',
      url='https://www.doc.ic.ac.uk/~slj12',
      packages=setuptools.find_packages(),
      package_data={'': ['*.ttm', '*.obj'],
                    'rlbench': ['task_design.ttt']},
      )
