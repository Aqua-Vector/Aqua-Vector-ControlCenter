# 협업 규칙

## 일과 사이클

| 시간 | 행동 |
|---|---|
| 18:00 전 | PR 오픈 (리뷰어 지정 필수) |
| 다음날 09:00 | develop 머지 |

---

## 브랜치

### 고정 브랜치

| 브랜치 | 용도 |
|---|---|
| `maater` | 릴리즈 전용 — 직접 push 금지 |
| `dev` | 통합 브랜치 — 매일 아침 머지 대상 |

### 작업 브랜치 명명

```
기능명/github id
```

소문자, 하이픈 구분

```
INS/ckstjrl
RC/cleanaftereat
```

---

## 커밋 메시지

```
type(scope): 한줄 요약 (50자 이내)

한 줄 요약은 영어로
```

### type

| type | 용도 |
|---|---|
| `feat` | 새 기능 추가 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변경 없는 리팩터 |
| `docs` | 주석 / README 변경 |
| `test` | 테스트 추가 / 수정 |
| `build` | CMake / Docker 변경 |
| `chore` | 설정 파일 등 잡무 |

### scope

`imu` `ins` `rs485` `servo` `nav` `guidance` `shared` `docker`

### 예시

```
feat(ins): add mahony complementary filter initialization routine
fix(rs485): fix uint32 RTT overflow
refactor(shared): replace shared buffer with lock-free atomic
docs(imu): add SPI DRDY interrupt timing comments
```

---

## PR 규칙

- 제목 형식: 커밋과 동일 — `feat(ins): ...` 대신 제목은 한글 가능
- 타겟: 항상 `develop` ← `feat/...` (main 직접 PR 금지)
- 리뷰: 꼭 리뷰를 길게 쓸 필요는 없지만 써줄 수 있다면 작성할 것
- 미완료 시: 제목에 `[WIP]` 접두사 + Draft PR

---

## 머지 / 기타

- **머지 방식**: Merge commit (Squash 금지 — 히스토리 보존)
- **최신화**: 작업 브랜치는 `git rebase develop`으로 동기화
- **충돌**: PR 올린 사람이 해결 후 re-push
- **shared_state.h 변경 시**: 반드시 상대방에게 사전 고지
- **항상 머지 후에는 pull 받기!!!**

---