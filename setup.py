import os
import re
import sys
import platform
from os import path
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
import distutils.cygwinccompiler
distutils.cygwinccompiler.get_msvcr = lambda: []

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.13.0':
                raise RuntimeError("CMake >= 3.13.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j4']

        cmake_args += ['-DBUILD_PYTHON_BINDINGS=ON']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''), self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', '--target', 'record3d'] + build_args, cwd=self.build_temp)


# read the contents of the README file
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md'), mode='rb') as f:
    long_description = f.read().decode('utf-8')

setup(
    name='record3d',
    version='1.4',
    license='lgpl-2.1',
    author='Marek Simonik',
    author_email='admin@record3d.app',
    url='https://github.com/marek-simonik/record3d',
    install_requires=[ 'numpy' ],
    keywords=['record3d', 'iOS', 'TrueDepth', 'streaming', 'pointcloud', 'point', 'cloud'],
    description='Accompanying library for the Record3D iOS app (https://record3d.app/). Allows you to receive RGBD stream from iOS devices with TrueDepth and/or LiDAR camera(s).',
    long_description=long_description,
    long_description_content_type='text/markdown',
    ext_modules=[CMakeExtension('record3d')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)