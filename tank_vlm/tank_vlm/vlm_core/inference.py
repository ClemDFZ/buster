"""Sync Qwen-VL engine for tank_vlm (llama-cpp multimodal)."""
from __future__ import annotations

import base64
import os
import threading
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from llama_cpp import Llama, LlamaGrammar, llama_chat_format

from tank_vlm.vlm_core.grounding import (
    DETECT_GBNF,
    GROUNDING_PROMPT,
    LetterboxMeta,
    crop_with_padding,
    encode_image,
    letterbox_square,
    norm1000_to_px,
    parse_bbox_response,
    px_to_norm,
    unletterbox_xyxy,
)

DEFAULT_N_CTX = 4096
DEFAULT_N_GPU_LAYERS = -1
DEFAULT_N_BATCH = 512
DEFAULT_N_UBATCH = 512
DEFAULT_IMAGE_MIN_TOKENS = 256
DEFAULT_IMAGE_MAX_TOKENS = 1024
DEFAULT_TEMPERATURE = 0.2
DEFAULT_TOP_P = 0.9
DEFAULT_MAX_TOKENS = 192

# Default model filenames living under tank_vlm/models/
DEFAULT_MODEL_ID = "qwen3-vl-2b"
DEFAULT_MODEL_FILE = "Qwen3-VL-2B-Instruct-Q4_K_M.gguf"
DEFAULT_CLIP_FILE = "Qwen3-VL-2B-Instruct-mmproj-F16.gguf"
DEFAULT_CHAT_FORMAT = "qwen2.5-vl"


class Qwen3VLChatHandler(llama_chat_format.Qwen25VLChatHandler):
    """Qwen2.5/3-VL handler with controllable image token budget (Jetson)."""

    def __init__(
        self,
        *args,
        image_min_tokens: int = DEFAULT_IMAGE_MIN_TOKENS,
        image_max_tokens: int = DEFAULT_IMAGE_MAX_TOKENS,
        vision_use_gpu: bool = False,
        **kwargs,
    ):
        self._img_min = int(image_min_tokens)
        self._img_max = int(image_max_tokens)
        self._vision_use_gpu = bool(vision_use_gpu)
        super().__init__(*args, **kwargs)

    def _init_mtmd_context(self, llama_model: Llama):
        if self.mtmd_ctx is not None:
            return
        from llama_cpp._utils import suppress_stdout_stderr
        import llama_cpp as _llama_cpp

        if self._vision_use_gpu and not os.environ.get("MTMD_BACKEND_DEVICE"):
            os.environ["MTMD_BACKEND_DEVICE"] = "CUDA0"

        with suppress_stdout_stderr(disable=self.verbose):
            ctx_params = self._mtmd_cpp.mtmd_context_params_default()
            ctx_params.use_gpu = self._vision_use_gpu
            ctx_params.image_min_tokens = self._img_min
            ctx_params.image_max_tokens = self._img_max
            ctx_params.print_timings = self.verbose
            ctx_params.n_threads = llama_model.n_threads
            ctx_params.flash_attn_type = _llama_cpp.LLAMA_FLASH_ATTN_TYPE_DISABLED
            self.mtmd_ctx = self._mtmd_cpp.mtmd_init_from_file(
                self.clip_model_path.encode(), llama_model.model, ctx_params
            )
            if self.mtmd_ctx is None:
                raise ValueError(f"Failed to load mtmd context from: {self.clip_model_path}")
            if not self._mtmd_cpp.mtmd_support_vision(self.mtmd_ctx):
                raise ValueError("Vision is not supported by this model")

            def mtmd_free():
                with suppress_stdout_stderr(disable=self.verbose):
                    if self.mtmd_ctx is not None:
                        self._mtmd_cpp.mtmd_free(self.mtmd_ctx)
                        self.mtmd_ctx = None

            self._exit_stack.callback(mtmd_free)


def _bgr_to_data_url(img_bgr, *, encode: str = "jpeg", jpeg_quality: int = 95) -> str:
    raw, mime = encode_image(img_bgr, encode=encode, jpeg_quality=jpeg_quality)
    b64 = base64.b64encode(raw).decode("ascii")
    return f"data:{mime};base64,{b64}"


class VLMEngine:
    """Thread-safe sync wrapper around llama-cpp multimodal."""

    def __init__(self) -> None:
        self._llm: Optional[Llama] = None
        self.model_path: Optional[str] = None
        self.clip_model_path: Optional[str] = None
        self.model_id: Optional[str] = None
        self.chat_format: Optional[str] = None
        self.n_ctx = DEFAULT_N_CTX
        self.n_gpu_layers = DEFAULT_N_GPU_LAYERS
        self.n_batch = DEFAULT_N_BATCH
        self.n_ubatch = DEFAULT_N_UBATCH
        self.image_min_tokens = DEFAULT_IMAGE_MIN_TOKENS
        self.image_max_tokens = DEFAULT_IMAGE_MAX_TOKENS
        self.vision_use_gpu = True
        self._detect_grammar: Optional[LlamaGrammar] = None
        self._lock = threading.Lock()

    @property
    def loaded(self) -> bool:
        return self._llm is not None

    def unload(self) -> None:
        with self._lock:
            self._unload_unlocked()

    def _unload_unlocked(self) -> None:
        old = self._llm
        self._llm = None
        self.model_path = None
        self.clip_model_path = None
        self.model_id = None
        self.chat_format = None
        self._detect_grammar = None
        if old is not None:
            del old

    def load(
        self,
        model_path: str | Path,
        clip_path: str | Path,
        *,
        model_id: str = DEFAULT_MODEL_ID,
        chat_format: str = DEFAULT_CHAT_FORMAT,
        n_ctx: int = DEFAULT_N_CTX,
        n_gpu_layers: int = DEFAULT_N_GPU_LAYERS,
        n_batch: int = DEFAULT_N_BATCH,
        n_ubatch: int = DEFAULT_N_UBATCH,
        image_min_tokens: int = DEFAULT_IMAGE_MIN_TOKENS,
        image_max_tokens: int = DEFAULT_IMAGE_MAX_TOKENS,
        vision_use_gpu: bool = True,
    ) -> None:
        p = str(model_path)
        c = str(clip_path)
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
        if not os.path.isfile(c):
            raise FileNotFoundError(c)

        self.image_min_tokens = int(image_min_tokens)
        self.image_max_tokens = int(image_max_tokens)
        self.vision_use_gpu = bool(vision_use_gpu)

        handler = Qwen3VLChatHandler(
            clip_model_path=c,
            verbose=False,
            image_min_tokens=self.image_min_tokens,
            image_max_tokens=self.image_max_tokens,
            vision_use_gpu=self.vision_use_gpu,
        )
        kwargs: dict[str, Any] = {
            "model_path": p,
            "n_ctx": int(n_ctx),
            "n_gpu_layers": int(n_gpu_layers),
            "n_batch": max(int(n_batch), int(n_ubatch)),
            "n_ubatch": int(n_ubatch),
            "verbose": False,
            "chat_handler": handler,
        }
        with self._lock:
            self._unload_unlocked()
            self._llm = Llama(**kwargs)
            self.model_path = p
            self.clip_model_path = c
            self.model_id = model_id
            self.chat_format = chat_format
            self.n_ctx = int(n_ctx)
            self.n_gpu_layers = int(n_gpu_layers)
            self.n_batch = int(n_batch)
            self.n_ubatch = int(n_ubatch)
            try:
                self._detect_grammar = LlamaGrammar.from_string(DETECT_GBNF, verbose=False)
            except Exception:
                self._detect_grammar = None
            # Eager mtmd init — fail at load, not mid-explore
            try:
                handler._init_mtmd_context(self._llm)
            except Exception as exc:
                self._unload_unlocked()
                raise RuntimeError(f"mtmd/vision init failed: {exc}") from exc

    def _get(self) -> Llama:
        if self._llm is None:
            raise RuntimeError("VLM not loaded")
        return self._llm

    def query(
        self,
        image_bgr: np.ndarray,
        question: str,
        *,
        temperature: float = DEFAULT_TEMPERATURE,
        top_p: float = DEFAULT_TOP_P,
        max_tokens: int = DEFAULT_MAX_TOKENS,
        encode: str = "jpeg",
        jpeg_quality: int = 95,
        grammar: Optional[LlamaGrammar] = None,
    ) -> str:
        with self._lock:
            return self._query_unlocked(
                image_bgr,
                question,
                temperature=temperature,
                top_p=top_p,
                max_tokens=max_tokens,
                encode=encode,
                jpeg_quality=jpeg_quality,
                grammar=grammar,
            )

    def _query_unlocked(
        self,
        image_bgr: np.ndarray,
        question: str,
        *,
        temperature: float = DEFAULT_TEMPERATURE,
        top_p: float = DEFAULT_TOP_P,
        max_tokens: int = DEFAULT_MAX_TOKENS,
        encode: str = "jpeg",
        jpeg_quality: int = 95,
        grammar: Optional[LlamaGrammar] = None,
    ) -> str:
        llm = self._get()
        image_url = _bgr_to_data_url(image_bgr, encode=encode, jpeg_quality=jpeg_quality)
        kwargs: dict[str, Any] = {
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": image_url}},
                        {"type": "text", "text": question},
                    ],
                }
            ],
            "temperature": temperature,
            "top_p": top_p,
            "max_tokens": max_tokens,
            "stream": False,
        }
        if grammar is not None:
            kwargs["grammar"] = grammar
        out = llm.create_chat_completion(**kwargs)
        return out["choices"][0]["message"]["content"]

    def _detect_once(
        self,
        image_bgr: np.ndarray,
        label: str,
        *,
        letterbox: bool,
        encode: str,
        jpeg_quality: int,
        max_tokens: int,
        use_grammar: bool,
    ) -> tuple[list[dict[str, Any]], str]:
        meta: Optional[LetterboxMeta] = None
        img = image_bgr
        if letterbox:
            img, meta = letterbox_square(image_bgr)
        prompt = GROUNDING_PROMPT.format(label=label)
        grammar = self._detect_grammar if use_grammar else None
        raw = self._query_unlocked(
            img,
            prompt,
            temperature=0.0,
            top_p=1.0,
            max_tokens=max_tokens,
            encode=encode,
            jpeg_quality=jpeg_quality,
            grammar=grammar,
        )
        parsed = parse_bbox_response(raw, coord_order="xyxy")
        src_h, src_w = image_bgr.shape[:2]
        out_h, out_w = img.shape[:2]
        boxes: list[dict[str, Any]] = []
        for item in parsed:
            px_model = norm1000_to_px(item["bbox_2d"], out_w, out_h)
            px_src = unletterbox_xyxy(px_model, meta) if meta is not None else px_model
            x1, y1 = min(px_src[0], px_src[2]), min(px_src[1], px_src[3])
            x2, y2 = max(px_src[0], px_src[2]), max(px_src[1], px_src[3])
            x1 = max(0.0, min(float(src_w), x1))
            y1 = max(0.0, min(float(src_h), y1))
            x2 = max(0.0, min(float(src_w), x2))
            y2 = max(0.0, min(float(src_h), y2))
            if (x2 - x1) < 4 or (y2 - y1) < 4:
                continue
            xyxy = [x1, y1, x2, y2]
            boxes.append(
                {
                    "xyxy_px": xyxy,
                    "xyxy_norm": px_to_norm(xyxy, src_w, src_h),
                    "label": item.get("label") or label,
                    "score": None,
                    "bbox_2d_1000": item["bbox_2d"],
                }
            )
        return boxes, raw

    def detect(
        self,
        image_bgr: np.ndarray,
        label: str,
        *,
        letterbox: bool = True,
        encode: str = "jpeg",
        jpeg_quality: int = 95,
        max_boxes: int = 4,
        max_tokens: int = 256,
        refine_passes: int = 0,
        refine_padding_ratio: float = 0.25,
        use_grammar: bool = True,
    ) -> dict[str, Any]:
        with self._lock:
            boxes, raw = self._detect_once(
                image_bgr,
                label,
                letterbox=letterbox,
                encode=encode,
                jpeg_quality=jpeg_quality,
                max_tokens=max_tokens,
                use_grammar=use_grammar,
            )
            n_refine = max(0, int(refine_passes))
            if n_refine > 0 and boxes:
                coarse = boxes[0]["xyxy_px"]
                samples = [coarse]
                for _ in range(n_refine):
                    patch, roi = crop_with_padding(
                        image_bgr, coarse, pad_ratio=refine_padding_ratio
                    )
                    if patch.size == 0:
                        break
                    local_boxes, _ = self._detect_once(
                        patch,
                        label,
                        letterbox=True,
                        encode=encode,
                        jpeg_quality=jpeg_quality,
                        max_tokens=max_tokens,
                        use_grammar=use_grammar,
                    )
                    if not local_boxes:
                        continue
                    lx1, ly1, lx2, ly2 = local_boxes[0]["xyxy_px"]
                    rx1, ry1, _, _ = roi
                    samples.append([rx1 + lx1, ry1 + ly1, rx1 + lx2, ry1 + ly2])
                if len(samples) > 1:
                    n = len(samples)
                    avg = [sum(s[i] for s in samples) / n for i in range(4)]
                    src_h, src_w = image_bgr.shape[:2]
                    boxes[0] = {
                        "xyxy_px": avg,
                        "xyxy_norm": px_to_norm(avg, src_w, src_h),
                        "label": boxes[0]["label"],
                        "score": None,
                        "bbox_2d_1000": None,
                        "refined": True,
                    }
            if max_boxes > 0:
                boxes = boxes[: int(max_boxes)]
            src_h, src_w = image_bgr.shape[:2]
            return {
                "boxes": boxes,
                "raw": raw,
                "input_size": [src_w, src_h],
                "model_id": self.model_id,
                "letterbox": bool(letterbox),
                "encode": encode,
            }

    def status(self) -> dict[str, Any]:
        return {
            "loaded": self.loaded,
            "model_id": self.model_id,
            "model_path": self.model_path,
            "clip_model_path": self.clip_model_path,
            "chat_format": self.chat_format,
            "vision_use_gpu": self.vision_use_gpu,
            "backend": "llama_cpp_inline",
        }
