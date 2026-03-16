import h5py
import json

# 파일 경로 설정
f = h5py.File(".hdf5 경로 넣기", "a") # 'a'는 수정 모드

# 전체 데모 목록 가져오기
all_demos = sorted(list(f["data"].keys()))

# 제외하고 싶은(안 좋은) 데모 목록
bad_demos = ["demo_3", "demo_5"] # 예시 번호입니다. 파악하신 번호로 수정하세요.

# 좋은 데모들만 리스트업
good_demos = [d for d in all_demos if d not in bad_demos]

# robomimic이 인식할 수 있는 Mask(Filter Key) 생성
if "mask" not in f:
    f.create_group("mask")

# 'better_demos'라는 이름의 필터 키로 저장
f["mask/better_demos"] = [n.encode("ascii") for n in good_demos]

f.close()
print('데모파일필터끝')