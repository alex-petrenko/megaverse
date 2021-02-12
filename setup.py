import os
import re
import sys
import platform
import subprocess
import multiprocessing

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                'CMake must be installed to build the following extensions: ' +
                ', '.join(e.name for e in self.extensions),
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = [
            f'-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}',
            f'-DPYTHON_EXECUTABLE={sys.executable}',
            # f'-DOpenCV_DIR =/home/alex/all/lib/opencv/build',  # TODO!!!
        ]

        # that's a hacky way to do it but the best idea I have at the moment
        bullet_root = os.environ.get('BULLET_ROOT', None)
        if bullet_root is not None:
            cmake_args.append(f'-DBULLET_ROOT={bullet_root}')

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if os.environ.get('VOXEL_WITH_GUI') == '1':
            build_gui = 'ON'
        else:
            build_gui = 'OFF'

        cmake_args += [f'-DCMAKE_BUILD_TYPE={cfg}', f'-DBUILD_GUI_APPS={build_gui}']
        build_args += ['--', f'-j{multiprocessing.cpu_count()}']

        env = os.environ.copy()

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        print('Build temp directory is ', self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(
            ['cmake', '--build', '.', '--target', 'voxel_env'] + build_args, cwd=self.build_temp,
        )

        print('Completed the build!')


def main():
    setup(
        name='voxel_env',
        version='0.0.1',
        author='Aleksei Petrenko',
        author_email='apetrenko1991@gmail.com',
        description='Fast immersive environment',
        long_description='',
        packages=find_packages(),
        ext_modules=[CMakeExtension('voxel_env.extension.voxel_env', 'src')],
        cmdclass=dict(build_ext=CMakeBuild),
        zip_safe=False,
    )

    return 0


if __name__ == '__main__':
    sys.exit(main())
