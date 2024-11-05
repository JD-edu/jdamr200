from setuptools import setup, find_packages

setup(
    name="jdamr200_lib",            # 패키지 이름
    version="0.1",                 # 패키지 버전
    packages=find_packages(),      # 모듈 내 패키지 자동 검색
    install_requires=[],           # 의존성 패키지 (필요시 추가)
    author="JD edu",            # 작성자 정보
    description="jdARM200 library",  # 모듈 설명
)
