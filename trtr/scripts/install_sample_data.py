#!/usr/bin/env python

from jsk_data import download_data

def main():
    PKG = 'trtr'

    ### OTB100 KiteSurf
    download_data(
        pkg_name=PKG,
        path='test_data/KiteSurf.zip',
        url='https://drive.google.com/uc?id=1mXxVnjQfTLNMCm-gTibvl-Y27q8sA0aO',
        md5='035b0fdf30357482edf1523e126da538',
        extract=True,
    )

    ### OTB100 Biker
    download_data(
        pkg_name=PKG,
        path='test_data/Biker.zip',
        url='https://drive.google.com/uc?id=1qIVvFDDGaGL2VFTJmBnYkbYD0twMksdV',
        md5='0c05d3d3ba6d450e8c7a8a9b304f86af',
        extract=True,
    )

    ### rosbag of jsk_perception
    download_data(
        pkg_name=PKG,
        path='sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vTDZOZGhrTTNvc00',
        md5='ad4e7d298c0d9985295d93e47c7b03e6',
        extract=False,
        compressed_bags=[
            'sample/data/2016-10-15-23-21-42_moving_bottle.bag',
        ],
    )


if __name__ == '__main__':
    main()
