#!/usr/bin/env python

from jsk_data import download_data

def main():
    PKG = 'trtr'

    ### KiteSurf
    download_data(
        pkg_name=PKG,
        path='test_data/KiteSurf.zip',
        url='https://drive.google.com/uc?id=1mXxVnjQfTLNMCm-gTibvl-Y27q8sA0aO',
        md5='035b0fdf30357482edf1523e126da538',
        extract=True,
    )

    ### Biker
    download_data(
        pkg_name=PKG,
        path='test_data/Biker.zip',
        url='https://drive.google.com/uc?id=1qIVvFDDGaGL2VFTJmBnYkbYD0twMksdV',
        md5='0c05d3d3ba6d450e8c7a8a9b304f86af',
        extract=True,
    )

if __name__ == '__main__':
    main()
