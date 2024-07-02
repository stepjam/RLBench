import codecs
import os.path
from setuptools import setup, find_packages


# Version meaning (X.Y.Z)
# X: Major version (e.g. vastly different scene, platform, etc)
# Y: Minor version (e.g. new tasks, major changes to existing tasks, etc)
# Z: Patch version (e.g. small changes to tasks, bug fixes, etc)


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()


def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith('__version__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")

core_requirements = [
    "pyrep @ git+https://github.com/stepjam/PyRep.git",
    "numpy",
    "Pillow",
    "pyquaternion",
    "scipy",
    "natsort"
]

setup(name='rlbench',
      version=get_version("rlbench/__init__.py"),
      description='RLBench',
      author='Stephen James',
      author_email='slj12@ic.ac.uk',
      url='https://www.doc.ic.ac.uk/~slj12',
      install_requires=core_requirements,
      packages=find_packages(),
      include_package_data=True,
      extras_require={
        "gym": ["gymnasium==1.0.0a2"],
        "dev": ["pytest"]
      },
      package_data={'': ['*.ttm', '*.obj', '**/**/*.ttm', '**/**/*.obj'],
                    'rlbench': ['task_design.ttt']},
      entry_points={
          "console_scripts": [
              "rlbench-generate-dataset = rlbench.dataset_generator:main"
          ]
      }
)
