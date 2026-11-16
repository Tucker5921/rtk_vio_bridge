from setuptools import setup
setup(name='rtk_vio_bridge', version='0.1.0', packages=['rtk_vio_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rtk_vio_bridge']),
                  ('share/rtk_vio_bridge', ['package.xml', 'README.md']),
                  ('share/rtk_vio_bridge/config', ['config/mower.yaml'])],
      install_requires=['setuptools', 'numpy'], tests_require=['pytest'], zip_safe=True,
      entry_points={'console_scripts': ['bridge = rtk_vio_bridge.node:main',
                                        'synthetic_rtk = rtk_vio_bridge.synthetic:main']})
