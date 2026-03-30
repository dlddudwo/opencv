# HALCON → OpenCV 4.6.0 포팅 가이드 (`opencv.cpp` 기준)

## 1) 기본 원칙

- **고정 버전**: OpenCV는 `4.6.0`으로 고정한다.
- **레시피 기반 파라미터 유지**: 기존 `a_recipe[...]` 접근 구조를 그대로 유지한다.
- **함수 단위 1:1 대응**: `Run`이 호출하는 하위 함수 단위(`GetStdDevImage`, `GetStdDevFilter` 등)로 포팅한다.
- **데이터 표현 분리**:
  - `cv::Mat gray`: grayscale image
  - `cv::Mat mask`: binary region(0/255)
  - 연결요소/shape filtering은 `connectedComponentsWithStats`, `findContours` 조합으로 처리

## 2) `opencv.cpp` 함수 체인 기준 포팅 순서

기존 체인:

1. `GetStdDevImage`
2. `GetStdDevFilter`
3. `GetStdDevFilterFillup`
4. `GetEnhanceOmitFilter` (Weak Scratch + Sticker + Dynamic)
5. `GetOmitImage`, `GetInspectionRoi`, `CalcOmitDefectData`

포팅도 동일 체인으로 진행한다. 먼저 1~3번(STDEV 라인)을 완성해 회귀 확인 후, 4번(복잡 로직)으로 확장한다.

---

## 3) HALCON ↔ OpenCV 매핑 (이 파일에서 실제 사용된 연산 기준)

- `Threshold(img, low, high)`
  - OpenCV: `cv::inRange(img, low, high, mask)`
- `Connection(region)`
  - OpenCV: `connectedComponentsWithStats(mask, labels, stats, centroids)`
- `SelectShape(..., "area", ..., min, max)`
  - OpenCV: stats의 `CC_STAT_AREA`로 필터 후 마스크 재구성
- `FillUp(region)`
  - OpenCV: contour flood fill/hole filling (`findContours(RETR_CCOMP)` + 내부 hole 채우기)
- `FillUpShape(..., "area", 1, fillup_area)`
  - OpenCV: hole contour area가 `fillup_area` 이하인 hole만 채우기
- `DilationCircle(radius)` / `ErosionCircle(radius)` / `ClosingCircle(radius)` / `OpeningCircle(radius)`
  - OpenCV: `getStructuringElement(MORPH_ELLIPSE, Size(k,k))` + `morphologyEx`
  - 커널 크기 예: `k = 2*round(radius)+1`
- `ScaleImage(scale, offset)`
  - OpenCV: `convertTo(dst, type, scale, offset)`
- `DeviationImage(maskX, maskY)`
  - OpenCV: local mean/mean of squares 기반 표준편차 계산
- `ReduceDomain(image, roiRegion)`
  - OpenCV: `bitwise_and(image, image, dst, roiMask)`
- `ZoomImageSize` / `ZoomRegion`
  - OpenCV: `resize`
- `Union1`, `Union2`, `ConcatObj`
  - OpenCV: `bitwise_or`
- `Intersection`
  - OpenCV: `bitwise_and`
- `GenRectangle1`
  - OpenCV: `rectangle`로 마스크 생성
- `SmallestRectangle1`
  - OpenCV: non-zero 픽셀의 bounding box (`boundingRect`)
- `DynThreshold(light)`
  - OpenCV: `src > (mean + offset)` 형태의 동적 threshold 구현
- `AreaCenter`
  - OpenCV: `countNonZero` + `moments`

> 주의: HALCON의 Region/XLD는 의미론이 OpenCV contour/mask와 완전히 동일하지 않으므로,
> 중간 단계마다 binary mask를 표준 표현으로 강제하는 것이 안정적이다.

---

## 4) `GetStdDevImage` 포팅 템플릿

핵심: HALCON `DeviationImage + ScaleImage` 동등 구현

1. 입력 grayscale `src`를 `CV_32F`로 변환
2. `blur(src)`로 지역 평균 `mean`
3. `blur(src^2)`로 `meanSq`
4. `stddev = sqrt(max(meanSq - mean*mean, 0))`
5. `convertTo(..., alpha=filter_scale)`로 스케일

이 함수는 이후 threshold 품질에 직접 영향하므로, **min/max 히스토그램 비교**를 HALCON 결과와 같이 검증한다.

---

## 5) `GetStdDevFilter` 포팅 템플릿

1. STDEV 이미지 threshold (`STDEV_THRESHOLD`)
2. 연결요소 area 필터(3 이상)
3. `DILATION_STDEV` 부호 기준으로 erosion/dilation 분기
4. 원본 threshold (`ORIGINAL_THRESHOLD`)
5. 연결요소 area 필터(3 이상)
6. dilation(`DILATION`)
7. 두 결과 OR 결합

실수 반경(radius)을 쓰던 HALCON과 달리 OpenCV는 커널 픽셀 단위이므로,
`radius -> kernel size` 변환 정책을 공통 유틸로 고정해야 결과 편차가 줄어든다.

---

## 6) `GetStdDevFilterFillup` 포팅 템플릿

- `FILLUP_MODE == 0`: 전체 hole fill
- 그 외: `FILLUP_AREA` 이하 hole만 fill

구현 팁:
- `findContours(mask, RETR_CCOMP, CHAIN_APPROX_SIMPLE)`에서 parent/child 계층 정보를 이용하면 hole만 선택적으로 채울 수 있다.

---

## 7) `GetEnhanceOmitFilter` 포팅 전략

이 함수는 3개의 브랜치가 혼합된다.

1. **Weak Scratch branch**
   - HALCON의 `EdgesSubPix`, `GenPolygonsXld`, `SplitContoursXld`, `UnionCollinearContoursXld`는 OpenCV에서 직접 1:1 대응이 약함
   - 권장 대체:
     - `Canny` + `HoughLinesP`(혹은 LSD) 기반 line segment 추출
     - 길이/각도/거리 조건으로 병합(커스텀)
     - 최종 `drawContours`/`line`으로 region mask화 후 dilation
2. **Sticker branch**
   - STDEV 기반 blob 추출 → closing → hole 수/area 조건 필터
   - `holes_num`은 contour hierarchy에서 child 개수로 계산
3. **Dynamic threshold branch (`GetSelectedRegion`)**
   - ROI 내부 `blur` 결과와 원본 비교로 `DynThreshold(light)` 구현

중요: 각 브랜치 출력은 항상 `CV_8U` mask(0/255)로 normalize 후 OR 결합한다.

---

## 8) `GetOmitImage` / `GetInspectionRoi` / `CalcOmitDefectData` 포팅 포인트

- `GetOmitImage`
  - region→binary, scale affine(`warpAffine`), blob filtering(area/ratio), 최종 binary 반환
- `GetInspectionRoi`
  - threshold 범위 → opening/closing → max area cluster 선택 → bounding rect → erosion
- `CalcOmitDefectData`
  - 면적 count: `countNonZero`
  - rate: `(resultArea / inspArea) * 10000`

예외 처리:
- 유효 region 없음 (`inspArea == 0` 등)일 때 0 반환 방어 코드를 공통화한다.

---

## 9) 구현 구조 권장안

- `opencv_porting_utils.h/.cpp` (공통 유틸)
  - `Mat thresholdRange(...)`
  - `Mat selectByArea(...)`
  - `Mat fillHoles(...)`
  - `Mat morphCircle(...)`
  - `int radiusToKernel(double r)`
- `omit_process_opencv.cpp`
  - HALCON 함수명과 같은 함수명으로 단계적 포팅
  - 각 단계별 디버그 dump 옵션(컴파일 플래그) 제공

---

## 10) 검증 방법 (권장)

- 동일 입력/recipe로 HALCON 결과와 OpenCV 결과를 비교:
  - IoU (mask overlap)
  - area 차이율
  - blob 개수
- 허용 오차를 단계별로 정의:
  - STDEV mask: IoU >= 0.97
  - 최종 Omit mask: IoU >= 0.95

---

## 11) 바로 적용 가능한 체크리스트

1. OpenCV 입력/출력을 모두 grayscale(`CV_8U`) + mask(`CV_8U`)로 정규화
2. morphology radius 변환 규칙 통일
3. 연결요소 area 필터 유틸 공통화
4. hole fill(전체/조건부) 유틸 분리
5. Weak Scratch는 별도 모듈로 분리(튜닝 포인트 많음)
6. 각 함수마다 HALCON 중간결과와 1:1 비교 이미지 저장

이 순서로 진행하면 `opencv.cpp`의 현재 HALCON 스타일 코드를 OpenCV 4.6.0 스타일로
안전하게 치환하면서 품질 회귀를 통제할 수 있다.
