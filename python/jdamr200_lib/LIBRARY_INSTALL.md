1. 디렉토리 구조 준비
우선 Python 모듈을 패키징할 디렉토리 구조를 준비합니다. 보통 다음과 같은 구조를 사용합니다.

bash
코드 복사
jdamr200_lib/
│
├── jdamr200_lib/           # 실제 코드가 들어가는 폴더
│   ├── __init__.py         # 모듈 초기화 파일 (필수)
│   └── jdamr200_lib.py     # 실제 Python 코드
│
├── setup.py                # 패키지 정보를 담은 파일 (필수)
└── README.md               # 패키지 설명 (선택)

2. setup.py 파일 작성
setup.py는 패키지를 설명하고, 설치하는 데 필요한 정보를 담은 파일입니다. 

from setuptools import setup, find_packages

setup(
    name="jdamr200_lib",            # 패키지 이름
    version="0.1",                 # 패키지 버전
    packages=find_packages(),      # 모듈 내 패키지 자동 검색
    install_requires=[],           # 의존성 패키지 (필요시 추가)
    author="JD edu",            # 작성자 정보
    description="jdARM200 library",  # 모듈 설명
)

3. 패키지 빌드
다음 명령을 사용하여 패키지를 빌드합니다. 우분투 터미널에서 패키지 폴더로 이동한 후 명령을 실행하세요.

4. __init__.py 파일 확인
jdamr200_lib 폴더 내에 __init__.py 파일이 있어야 합니다. 이 파일이 없으면 Python은 해당 폴더를 패키지로 인식하지 못합니다.

jdamr200_lib/__init__.py 예시:

from .your_script import Jdamr200  

pip install setuptools wheel  # 필요한 도구 설치
python setup.py sdist bdist_wheel

5. PIP로 패키지 설치
패키지를 로컬에서 설치하려면 생성된 파일을 PIP로 설치합니다.

pip install dist/jdamr200_lib-0.1-py3-none-any.whl --force-reinstall



