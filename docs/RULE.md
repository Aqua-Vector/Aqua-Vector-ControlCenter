# 협업 규칙

## type

| type | 용도 |
|---|---|
| `feat` | 새 기능 추가 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변경 없는 리팩터 |
| `docs` | 주석 / README 변경 |
| `test` | 테스트 추가 / 수정 |
| `build` | CMake / Docker 변경 |
| `chore` | 설정 파일 등 잡무 |

## 이슈

### 이슈 작성 명명
```
[TYPE] 내용...
type은 대문자로 작성
```
#### 예시

```
FEAT: 모터 제어 기능
```   

<br>

## 브랜치

### 고정 브랜치

| 브랜치 | 용도 |
|---|---|
| `main` | 릴리즈 전용 — 직접 push 금지 |   

### 작업 브랜치 명명

```
type/#{issue number}
```
#### 예시

```
feature/#1
```

---

## 커밋 메시지

```
type: 한줄 요약 (50자 이내)

한 줄 요약은 영어로
```
#### 예시

```
feat: add mahony complementary filter initialization routine
fix: fix uint32 RTT overflow
refactor: replace shared buffer with lock-free atomic
docs: add SPI DRDY interrupt timing comments
```

---

## PR 규칙

- 제목 형식: 커밋과 동일 — `feat: ...` 대신 제목은 한글 가능
- 타겟: 항상 `main` ← `feat/...` (main 직접 PR 금지)

---

## 머지 / 기타

- **충돌**: PR 올린 사람이 해결 후 re-push
- **항상 머지 후에는 pull 받기!!!**

---