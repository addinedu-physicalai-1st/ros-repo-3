"""
SAC Actor → ONNX 내보내기 스크립트

학습된 SAC 체크포인트(network.th)에서 Actor 가중치만 추출하여
추론 전용 ONNX 모델로 변환한다.

ONNX 모델 사양:
    Input:  state  (1, 28) float32
    Output: action (1, 2)  float32  — tanh(mean), 범위 [-1, 1]

사용법:
    python export_onnx.py \
        --checkpoint results/pinky_sac/models/.../225280/network.th \
        --output models/sac_actor.onnx
"""

import argparse
import sys
from pathlib import Path

import torch
import torch.nn as nn
import numpy as np


# ──────────────────────────────────────────────
# 1. 네트워크 정의 (sac_network.py와 동일 구조)
# ──────────────────────────────────────────────

LOG_STD_MIN = -20
LOG_STD_MAX = 2


class SACActorNet(nn.Module):
    """Squashed Gaussian 정책 네트워크 (학습 코드와 동일 구조)."""

    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
        )
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)

    def forward(self, state: torch.Tensor):
        x = self.net(state)
        mean = self.mean_head(x)
        log_std = self.log_std_head(x)
        log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        return mean, log_std


class DeterministicActor(nn.Module):
    """추론 전용 래퍼: state → tanh(mean)."""

    def __init__(self, actor: SACActorNet):
        super().__init__()
        self.actor = actor

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        mean, _ = self.actor(state)
        return torch.tanh(mean)


# ──────────────────────────────────────────────
# 2. 체크포인트에서 Actor 가중치 추출
# ──────────────────────────────────────────────

def extract_actor_weights(checkpoint_path: str) -> dict:
    """network.th에서 actor.* 키만 추출하고 접두사를 제거한다."""
    full_state = torch.load(checkpoint_path, map_location="cpu", weights_only=True)

    actor_state = {}
    for key, value in full_state.items():
        if key.startswith("actor."):
            actor_state[key[len("actor."):]] = value

    if not actor_state:
        print("[ERROR] 체크포인트에서 'actor.*' 키를 찾을 수 없습니다.")
        print(f"  발견된 키: {list(full_state.keys())[:10]}...")
        sys.exit(1)

    return actor_state


# ──────────────────────────────────────────────
# 3. ONNX 내보내기
# ──────────────────────────────────────────────

def export_to_onnx(model: nn.Module, output_path: str,
                   state_dim: int, opset_version: int = 17):
    """PyTorch 모델을 ONNX로 내보낸다."""
    import onnx

    model.eval()
    dummy_input = torch.randn(1, state_dim)

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        opset_version=opset_version,
        input_names=["state"],
        output_names=["action"],
        dynamic_axes={
            "state": {0: "batch_size"},
            "action": {0: "batch_size"},
        },
    )

    # 외부 데이터 파일이 생성된 경우, 단일 파일로 합치기
    external_data = Path(output_path + ".data")
    if external_data.exists():
        onnx_model = onnx.load(output_path, load_external_data=True)
        onnx.save(onnx_model, output_path)
        external_data.unlink()
        print(f"[OK] ONNX 내보내기 완료 (단일 파일): {output_path}")
    else:
        print(f"[OK] ONNX 내보내기 완료: {output_path}")


# ──────────────────────────────────────────────
# 4. 검증
# ──────────────────────────────────────────────

def validate_onnx(onnx_path: str):
    """ONNX 모델 구조 검증."""
    try:
        import onnx
        model = onnx.load(onnx_path)
        onnx.checker.check_model(model)
        print("[OK] ONNX 모델 구조 검증 통과")
    except ImportError:
        print("[SKIP] onnx 패키지 미설치 — 구조 검증 생략")
    except Exception as e:
        print(f"[ERROR] ONNX 모델 검증 실패: {e}")
        sys.exit(1)


def compare_outputs(pytorch_model: nn.Module, onnx_path: str,
                    state_dim: int, num_tests: int = 100,
                    tolerance: float = 1e-5):
    """PyTorch vs ONNX Runtime 출력 수치 비교."""
    try:
        import onnxruntime as ort
    except ImportError:
        print("[SKIP] onnxruntime 미설치 — 수치 비교 생략")
        return

    session = ort.InferenceSession(onnx_path)
    pytorch_model.eval()

    max_diff = 0.0
    for i in range(num_tests):
        test_input = np.random.randn(1, state_dim).astype(np.float32)

        # PyTorch 추론
        with torch.no_grad():
            pt_output = pytorch_model(
                torch.from_numpy(test_input)).numpy()

        # ONNX Runtime 추론
        ort_output = session.run(
            None, {"state": test_input})[0]

        diff = np.max(np.abs(pt_output - ort_output))
        max_diff = max(max_diff, diff)

        if diff > tolerance:
            print(f"[WARN] 테스트 #{i}: 최대 오차 {diff:.2e} > {tolerance:.2e}")
            print(f"  입력:       {test_input[0, :5]}...")
            print(f"  PyTorch:    {pt_output[0]}")
            print(f"  ONNX:       {ort_output[0]}")

    if max_diff <= tolerance:
        print(f"[OK] {num_tests}회 수치 비교 통과 (최대 오차: {max_diff:.2e})")
    else:
        print(f"[WARN] 최대 오차 {max_diff:.2e} — "
              f"허용치 {tolerance:.2e} 초과 (부동소수점 차이일 수 있음)")


# ──────────────────────────────────────────────
# 5. 모델 정보 출력
# ──────────────────────────────────────────────

def print_model_info(actor_state: dict, state_dim: int,
                     action_dim: int, hidden_dim: int):
    """추출된 가중치 정보를 출력한다."""
    print("=" * 60)
    print("SAC Actor 모델 정보")
    print("=" * 60)
    print(f"  State dim:  {state_dim}")
    print(f"  Action dim: {action_dim}")
    print(f"  Hidden dim: {hidden_dim}")
    print(f"  추출된 키:")
    total_params = 0
    for key, tensor in actor_state.items():
        num = tensor.numel()
        total_params += num
        print(f"    {key:30s}  {str(list(tensor.shape)):>20s}  ({num:,} params)")
    print(f"  총 파라미터: {total_params:,}")
    print("=" * 60)


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SAC Actor를 ONNX로 내보내기")
    parser.add_argument(
        "--checkpoint", required=True,
        help="network.th 체크포인트 경로")
    parser.add_argument(
        "--output", default="models/sac_actor.onnx",
        help="출력 ONNX 파일 경로 (기본: models/sac_actor.onnx)")
    parser.add_argument(
        "--state-dim", type=int, default=28,
        help="관측 벡터 차원 (기본: 28)")
    parser.add_argument(
        "--action-dim", type=int, default=2,
        help="액션 벡터 차원 (기본: 2)")
    parser.add_argument(
        "--hidden-dim", type=int, default=256,
        help="은닉층 크기 (기본: 256)")
    parser.add_argument(
        "--skip-validation", action="store_true",
        help="ONNX 검증 및 수치 비교 건너뛰기")
    args = parser.parse_args()

    # 1. Actor 가중치 추출
    print(f"\n[1/4] 체크포인트 로드: {args.checkpoint}")
    actor_state = extract_actor_weights(args.checkpoint)
    print_model_info(actor_state, args.state_dim,
                     args.action_dim, args.hidden_dim)

    # 2. Actor 네트워크 재구성 및 가중치 로드
    print("[2/4] Actor 네트워크 재구성...")
    actor = SACActorNet(args.state_dim, args.action_dim, args.hidden_dim)
    actor.load_state_dict(actor_state)
    actor.eval()

    # 추론 래퍼로 감싸기
    deterministic_actor = DeterministicActor(actor)
    deterministic_actor.eval()

    # 3. ONNX 내보내기
    print(f"[3/4] ONNX 내보내기: {args.output}")
    export_to_onnx(deterministic_actor, args.output, args.state_dim)

    # 4. 검증
    if not args.skip_validation:
        print("[4/4] 검증 중...")
        validate_onnx(args.output)
        compare_outputs(deterministic_actor, args.output, args.state_dim)
    else:
        print("[4/4] 검증 생략")

    print(f"\n완료! ONNX 모델: {args.output}")
    print("이 파일을 로봇에 복사하여 ONNX Runtime으로 추론하세요.")


if __name__ == "__main__":
    main()
