#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'trtr'

    download_data(
        pkg_name=PKG,
        path='models/trtr_resnet50_encoder.onnx',
        url='https://drive.google.com/uc?id=1JxNXTATswhCCNxDo1vNvSQJd6LwxQ_jJ',
        md5='0269312056510c2bb58045748c25ecbb',
        extract=False,
    )

    download_data(
        pkg_name=PKG,
        path='models/trtr_resnet50_decoder.onnx',
        url='https://drive.google.com/uc?id=1FJqk2povbJh-oqNHSDvOwFxzyzZSqNSb',
        md5='c0defc4493077a579fc4a5308dc85939',
        extract=False,
    )

if __name__ == '__main__':
    main()
