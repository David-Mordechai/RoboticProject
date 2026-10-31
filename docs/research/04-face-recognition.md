# 04: Face recognition (who is this person)

*Research date: 2026-10-31. This answers open issue **#13** and the face part of plan **§13** (deliver a message, greet by name, recognize the owner on "hungry"). It builds on [03-sensors-mapping-multirobot.md](03-sensors-mapping-multirobot.md) §1.2 (HP60C) and §2. The cars haven't arrived, so nothing here is measured yet. Every number is either cited, or marked as derived (*derived*) or as an *estimate*.*

**Key sources**

| Short name | What |
|---|---|
| [GH] | Hiwonder `MentorPi-A1` code, cloned to `docs/vendor/hiwonder/code/MentorPi-A1/` |
| [HW-A1] | HP60C specs as recorded in 03 §1.2 (product page, re-read 2026-10-31) |
| [IF-ZOO] | InsightFace model zoo, https://github.com/deepinsight/insightface/tree/master/model_zoo (read 2026-10-31) |
| [IF-LIC] | InsightFace README "License", https://github.com/deepinsight/insightface |
| [IF-GUIDE] | InsightFace guide, https://insightface.ai/guides/choose-face-recognition-model-and-evaluate |
| [ADA] | AdaFace, Kim, Jain, Liu, CVPR 2022, arXiv 2204.00964; README https://github.com/mk-minchul/AdaFace |
| [CVLF] | CVLFace (the new AdaFace repo), https://github.com/mk-minchul/CVLFace; HF card https://huggingface.co/minchul/cvlface_adaface_vit_base_kprpe_webface4m |
| [EDGE] | EdgeFace, George et al., arXiv 2307.01838; https://github.com/otroshi/edgeface (+ LICENSE) |
| [GFN] | GhostFaceNets, https://github.com/HamadYA/GhostFaceNets |
| [SFACE] | OpenCV Zoo SFace, https://github.com/opencv/opencv_zoo/tree/main/models/face_recognition_sface |
| [YUNET] | OpenCV Zoo YuNet, https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet |
| [OCV-BENCH] | OpenCV Zoo benchmark table, https://github.com/opencv/opencv_zoo/blob/main/benchmark/README.md |
| [DLIB] | https://github.com/davisking/dlib-models and https://github.com/ageitgey/face_recognition |
| [DEEPF] | DeepFace wrapper, https://github.com/serengil/deepface |
| [MP] | MediaPipe Face Detector, https://developers.google.com/edge/mediapipe/solutions/vision/face_detector and legacy doc https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/face_detection.md |
| [HAILO-MZ] | Hailo Model Zoo, Hailo-8L face recognition table, https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8L/HAILO8L_face_recognition.rst |
| [HAILO-APPS] | https://github.com/hailo-ai/hailo-apps; Seeed demo https://wiki.seeedstudio.com/facial_recognition_on_the_recomputer/ |
| [RPI-AIHAT] | https://www.raspberrypi.com/products/ai-hat/; AI HAT+ 2: https://www.hackster.io/news/gen-ai-on-your-raspberry-pi-a-hands-on-review-of-the-raspberry-pi-ai-hat-2-3c829a8894dd, https://heise.de/-11136612 |
| [RPI5] | https://www.raspberrypi.com/news/introducing-raspberry-pi-5/ |
| [RPI-CAM] | https://www.raspberrypi.com/products/camera-module-3/ and https://www.raspberrypi.com/documentation/accessories/camera.html |
| [TINY] | TinyFace, Cheng et al., arXiv 1811.08965 |
| [SURV] | QMUL-SurvFace, Cheng et al., arXiv 1804.09691; benchmark https://qmul-survface.github.io/benchmark.html |
| [AWS-CAM] | AWS Rekognition camera and input-image recommendations, https://docs.aws.amazon.com/rekognition/latest/dg/recommendations-camera-image-video.html and `.../recommendations-facial-input-images.html` |
| [ICAO] | ICAO TAG-MRTD/17 WP009, https://www2023.icao.int/Meetings/TAG-MRTD/Documents/Tag-Mrtd-17/TagMrtd17_WP009.pdf (ISO/IEC 19794-5 minimum 90 px between eye centres) |
| [DORI] | IEC EN 62676-4:2015 DORI levels, as quoted by https://www.cctvdesigntool.com/blog/dori-standard-cctv/ (standard text not read) |
| [PEEP] | "Peepers & Pixels", arXiv 2503.20108 (human recognition vs IPD) |
| [POSE] | Grimmer, Rathgeb, Busch, "Pose Impact Estimation on Face Recognition…", arXiv 2303.00491 |
| [IPD] | US Army ANSUR II 2012 (Gordon et al. 2014), as tabulated in https://en.wikipedia.org/wiki/Pupillary_distance |
| [ANTH] | CityU online anthropometry: eye height https://personal.cityu.edu.hk/meachan/online%20anthropometry/Chapter2/Ch2-2.htm, sitting eye height `Ch2-9.htm`, popliteal height `Ch2-16.htm` |
| [NIST-1N] | NIST FRTE 1:N, https://pages.nist.gov/frvt/html/frvt1N.html |
| [CRFIQA] | CR-FIQA, Boutros et al., CVPR 2023, https://github.com/fdbtrs/cr-fiqa |
| [REID] | torchreid / OSNet model zoo, https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO (MIT license) |
| [LVLM-REID] | Hambarde, Samale, Proença, "Human Re-ID Meets LVLMs", arXiv 2501.18698 |
| [IL-PPL] | Israel Privacy Protection Law, Amendment 13: DLA Piper, https://www.dlapiperdataprotection.com/?c=IL; PPA workplace biometrics guideline, https://www.biometricupdate.com/202403/israel-bosses-can-only-collect-biometrics-if-they-heed-worker-privacy |
| [GDPR] | GDPR Art. 9, https://gdpr-info.eu/art-9-gdpr/ |
| [AIA] | EU AI Act Art. 5, https://artificialintelligenceact.eu/article/5/ |

---

## 0. TL;DR

- **The geometry is the hard part, not the model.** With the HP60C at ≈0.12 m and no tilt, a *standing* person's eyes enter the frame only beyond **≈2.9–3.3 m**; a seated person's beyond **≈2.1–2.3 m** (*derived*, §2.2). At those distances the stock 640×480 stream gives an eye distance of only **≈6–10 px** (*derived*). The "faces at 1–4 m" assumption doesn't hold for standing people at 1–2 m: they're above the frame.
- **The pitch limit bounds the distance from below anyway.** Sources put the useful limit at ≈30° pitch [POSE] (AWS: <45° looking up [AWS-CAM]). From a 0.12 m camera that means **≥≈2.4–2.8 m** for standing people and **≥≈1.7–2.0 m** for seated (*derived*). So recognition happens at **2–4 m**, which needs far more pixels than 640×480 gives.
- **Hiwonder's stock detector is the wrong one.** Its `detector.tflite` is 229,746 bytes, the same size as MediaPipe's *short-range* BlazeFace. Google documents that model "for faces within 2 meters" [MP], [GH].
- **Models:** the best *low-resolution* results in open source are AdaFace / CVLFace (TinyFace rank-1 **72–76 %** [CVLF]). InsightFace `buffalo_l` is strong on high-quality benchmarks (IJB-C 97.25 %) [IF-ZOO]. **Licence is the blocker:** InsightFace weights are "non-commercial research purposes only" [IF-LIC]; AdaFace/CVLFace weights say "follow the license of the training dataset" (WebFace4M/12M, research-only) [CVLF]. **Cleanly licensed:** OpenCV SFace (Apache 2.0, 99.40 % on its eval) [SFACE], dlib (CC0, LFW 99.38 %) [DLIB], EdgeFace (BSD-3 code; weights trained on WebFace subsets) [EDGE].
- **Recommended pipeline (§7):**
  1. The car finds people with person detection and **stops** to look (no motion blur).
  2. It captures a **high-resolution frame**: HP60C at 1920×1080 if the driver supports it, or better, a cheap upward-tilted **Raspberry Pi Camera Module 3** still. That gives an eye distance of ≈56–89 px at 2.5–4 m (*derived*).
  3. A light face detector on the Pi (YuNet, MIT) finds the face; the Pi sends **face crops** (kilobytes, not video) to the GB10.
  4. The GB10 aligns, quality-scores and embeds the face, pools the frames of one track into a template, and does **open-set matching** with a threshold calibrated on our own people.
  5. Person re-ID (OSNet) holds the identity when the face turns away. A VLM is for descriptions only, not for identity.
- **Privacy:** face templates are biometric data. In Israel that is "especially sensitive" data under Amendment 13 (in force since 2025-08-14) [IL-PPL]. GDPR Art. 9 treats it the same way, with explicit consent as the basis [GDPR]. Keep it local, consent-based, minimal and deletable.

## 1. Candidate recognition models (2026)

Accuracy figures are the authors' own; TAR is at FAR = 1e-4 unless noted. "–" means no source gives the value.

| Model | Params / size | LFW | IJB-B / IJB-C | Low-res (TinyFace rank-1) | Speed on ARM / edge | Code licence | Weights licence |
|---|---|---|---|---|---|---|---|
| **InsightFace `buffalo_l`** (SCRFD-10GF + ResNet50 @ WebFace600K) | pack 326 MB | 99.83 | – / 97.25 | – | – | MIT | **"non-commercial research purposes only"**; commercial licence by email [IF-LIC] |
| InsightFace `buffalo_s`/`_sc` (SCRFD-500MF + MBF @ WebFace600K) | 159 MB / **16 MB** | 99.70 | – / 95.02 | – | – | MIT | Same as above |
| InsightFace `antelopev2` (ResNet100 @ Glint360K) | 407 MB | – | – | – | – | MIT | Same as above [IF-ZOO] |
| **AdaFace** IR101 @ WebFace12M | – | 99.82 | 96.41 / 97.66 | **72.29** | – | MIT | Not stated in the README [ADA]; CVLFace cards: "follow the license of the training dataset" [CVLF] |
| AdaFace IR50 @ WebFace4M / IR18 @ WebFace4M | – | 99.78 / 99.53 | 95.44 / 96.98; 93.03 / 94.99 | 70.20 / 66.07 [CVLF] | – | MIT | As above |
| **CVLFace ViT KP-RPE** (AdaFace) @ WebFace12M | – | – | 96.55 / 97.82 ("@0.01", as printed) | **76.10** | – | MIT | As above [CVLF] |
| **EdgeFace-XS (γ=0.6)** | 1.77 M params, 154 MFLOPs | 99.73 | 92.67 / 94.85 | – | No timing given [EDGE] | BSD-3 [EDGE] | Not separately stated; trained on WebFace4M/12M subsets [EDGE paper] |
| EdgeFace-S (γ=0.5) / base | 3.65 M / 18.23 M | 99.78 / 99.83 | 93.58 / 95.63 (S) | – | – | BSD-3 | As above |
| GhostFaceNetV1-1.3-1 @ MS1MV3 | – | 99.73 | 93.12 / 94.94 | – | – | MIT [GFN] | Trained on MS1MV2/V3 (InsightFace data: research-only [IF-LIC]) |
| **OpenCV SFace** (+ YuNet detector) | – | 0.9940 on OpenCV's eval (int8: 0.9932) | – | – | **68.82 ms** on a Pi 4B CPU; 28.94 ms on RK3588 CPU [OCV-BENCH] | Apache 2.0 | **Apache 2.0** [SFACE] |
| dlib ResNet (`face_recognition`) | – | 99.38 | – | – | Pi install guide, no timing [DLIB] | MIT (wrapper) | **CC0**; trained on ≈3 M faces from FaceScrub, VGG and web scrapes [DLIB] |
| Hailo `arcface_mobilefacenet` | 2.04 M, 0.88 GOPs, 112×112 | 99.4 (also after quantization) | – | – | **1666 fps** on Hailo-8L (host x86, PCIe Gen3 x4) [HAILO-MZ] | MIT (zoo) | Origin of the weights not stated; likely InsightFace-derived (*unverified*) |
| DeepFace (wrapper) | – | – | – | – | – | MIT | "Wrapped models inherit their own licences" [DEEPF] |
| FaceNet | – | – | – | – | – | – | Not researched further: superseded by the rows above |

**Notes**
- **High-quality benchmarks are saturated:** LFW 99.4–99.8 % for all serious models [IF-GUIDE]. They don't tell us how a model copes with our setup. **TinyFace** (faces 6–32 px tall, average 20 [TINY]) and **QMUL-SurvFace** (average ≈24×20 px [SURV]) are the relevant tests, and only AdaFace/CVLFace publish TinyFace numbers. No source gives TinyFace, SCface or SurvFace numbers for SFace, EdgeFace, buffalo or dlib.
- **SurvFace shows how hard native low-res is.** On native surveillance faces, the best baseline in the paper reaches only **29.9 %** closed-set rank-1 (65.2 % on MegaFace). Open-set: TPIR **13.8 %** at FPIR 10 % [SURV]. Those are 2018 models, but they show why our pipeline must avoid producing faces that small.
- **Licence summary.** Practically every high-accuracy open model was trained on research-only data: WebFace260M subsets, MS1M and Glint360K. InsightFace says so explicitly for its packs [IF-LIC].
  - "Non-commercial research" is **not obviously** our use: an office robot at a company. Personal hobby use isn't addressed by any of these texts.
  - **Decision for the user:** either (a) accept the research-only weights for a non-commercial internal prototype after checking with legal, (b) buy InsightFace's commercial licence, or (c) use SFace/dlib (clean licences, but no low-res data published).

## 2. The hard parts for our camera

### 2.1 How many pixels a face needs

| Source | Requirement |
|---|---|
| ISO/IEC 19794-5 (passport-grade enrolment) | ≥ **90 px** between eye centres [ICAO] |
| InsightFace commercial-upgrade trigger | "low resolution under ~**48 px** inter-pupillary distance" is where open models struggle [IF-GUIDE] |
| AWS Rekognition | Face ≥ **50×50 px** in images up to 1920×1080; up-sampling "does not produce the results needed" [AWS-CAM] |
| IEC 62676-4 DORI | Identification **250 px/m**, recognition 125 px/m [DORI] |
| Human observers | Accuracy at chance around **10 px** IPD [PEEP] |

### 2.2 What our cameras give (*derived; estimates until measured*)

- **Formula.** Focal length in pixels: f = (W/2) / tan(HFOV/2). Eye distance in pixels: IPD_px = f · IPD / d, with IPD ≈ 63 mm (ANSUR II means: 61.7 mm female, 64.0 mm male [IPD]). Pixels per metre: f / d.
- **How the 640×480 stream maps onto the sensor is unknown.** The HFOV 80.9° × VFOV 51.7° spec [HW-A1] fits a 16:9 image. The stock 640×480 [GH] is 4:3. Two cases:
  - **"scaled":** the full 80.9° squeezed into 640 px, f ≈ 375 px.
  - **"crop":** a 4:3 centre crop scaled down, HFOV ≈ 65.2°, f ≈ 500 px.
  - Read `camera_info` to find out which.

| Stream | f (px) | IPD px @ 1 / 2 / 2.5 / 3 / 4 m | px/m @ 1 / 2 / 3 / 4 m |
|---|---|---|---|
| HP60C 640×480 "scaled" (stock) | 375 | 23.6 / 11.8 / 9.5 / 7.9 / 5.9 | 375 / 188 / 125 / 94 |
| HP60C 640×480 "crop" | 500 | 31.5 / 15.8 / 12.6 / 10.5 / 7.9 | 500 / 250 / 167 / 125 |
| HP60C 1920×1080 (max, ≤20 fps [HW-A1]) | 1126 | 70.9 / 35.5 / 28.4 / 23.6 / 17.7 | 1126 / 563 / 375 / 282 |
| Pi Camera Module 3, 1080p video (HFOV 66° [RPI-CAM]) | 1478 | 93 / 47 / 37 / 31 / 23 | 1478 / 739 / 493 / 370 |
| Pi Camera Module 3, 12 MP still (4608 px wide [RPI-CAM]) | 3548 | 224 / 112 / **89** / 75 / **56** | 3548 / 1774 / 1183 / 887 |

- **Is the face even in the frame?** The camera sits at ≈0.12 m with no tilt (03 §1.2), so the top of the image is 25.85° above horizontal (VFOV/2). Eyes enter the frame only at d ≥ (h_eye − 0.12) / tan 25.85°.
  - **Eye heights [ANTH]:** standing, 50th percentile, 1505 mm (British women) to 1710 mm (US men). Seated ≈ sitting eye height 720–800 mm + popliteal (seat) height 400–445 mm ≈ 1.12–1.25 m (*estimate*; shoes and chair settings ignored).
  - **Standing:** eyes enter the frame at **2.9–3.3 m** (3.5 m for a 95th-percentile US man).
  - **Seated:** **2.1–2.3 m**.
  - So at 640×480, a standing person is in frame only where the eye distance is ≈6–10 px. That is below the human chance level [PEEP] and in SurvFace territory [SURV].
- **Pitch.** Above ≈30° pitch, face recognition degrades significantly [POSE]. AWS recommends <45° when the camera looks up, and an overall pitch ≤30° [AWS-CAM]. Keeping the elevation ≤30° from a 0.12 m camera means d ≥ **2.4–2.8 m** standing and **1.7–2.0 m** seated (*derived*). That's true for any camera at this height. Tilting the camera up doesn't fix pitch; it only brings the face into the frame.
- **Conclusion.** The working distance is **≈2–4 m**. In that range the HP60C at 1920×1080 gives an eye distance of ≈18–35 px: low-res territory, so use AdaFace-class models and multi-frame pooling. A Module 3 still gives ≈56–112 px, near or above the 48–90 px guidance. A mast that raises the camera would reduce both problems; no source sizes that for this car.
- **HP60C 1080p over USB 2.0.** Raw YUYV at 1920×1080×2 B×20 fps is ≈83 MB/s (*derived*). That exceeds USB 2.0's 480 Mbit/s (60 MB/s) signalling rate before depth is even counted. So 1080p must be compressed or run at lower fps. No source says which the `ascamera` driver does. **Test it.**

### 2.3 Motion blur, lighting, super-resolution

- **Blur:** use sharp images and avoid subject and camera motion [AWS-CAM]. NIST's video evaluation names motion blur and occlusion as what makes face tracking hard (NIST IR 8173, via https://www.nist.gov/news-events/news/2017/04/identifying-faces-video-images-major-challenge-nist-report-shows). The HP60C RGB exposure time is unpublished. **Behavioural fix:** stop the car before looking.
- **Lighting:** prefer flat lighting without shadows [AWS-CAM]. A low camera looks up into ceiling lights (backlight); no source quantifies the effect. **Measure it in our office.**
- **Super-resolution doesn't help.** On native low-res faces, SR "is ineffective" and "often bring[s] slightly negative effect" [SURV]. Up-sampling "does not produce the results needed" [AWS-CAM]. Interpolation fails to recover biometric features (FG 2025, "Reading Between the Eyes", abstract only). **Get real pixels instead.**
- **Quality filtering.** AdaFace's feature norm approximates image quality [ADA], so it's free with that model. CR-FIQA is a dedicated quality score, but it is **CC BY-NC 4.0** [CRFIQA]. Use quality to drop frames and to weight template pooling.

## 3. Where to run what

| Stage | Where | Why / source |
|---|---|---|
| Person detection ("is someone there") | Pi | Hiwonder already ships YOLO (03 §2.1); needed to steer and stop the car |
| Face detection | Pi | YuNet: MIT, detects faces "around 10×10 to 300×300" px, **6.23 ms** at 160×120 on a Pi 4B [YUNET], [OCV-BENCH]. Not MediaPipe short-range (≤2 m [MP]); use full-range (≤5 m [MP]) if we stay with MediaPipe |
| Face crop + metadata → GB10 | Wi-Fi | A 112×112 RGB crop is ≈38 kB raw (*derived*); a few crops/s is ≈0.1–0.2 MB/s. Full 640×480 frames are 13.8 MB/s raw (03 §3.3), and the Pi 5 has no H.264 encoder (03, F12) |
| Alignment, quality, embedding, matching | GB10 | Heavy models (IR101/ViT) fit there; one gallery for both cars. No GB10 timing published: **measure it** |
| Fallback when Wi-Fi drops | Pi | SFace ≈69 ms on a Pi 4B [OCV-BENCH]. The Pi 5 is "2–3×" a Pi 4 (CPU+GPU claim [RPI5]), so ≈23–35 ms per face on a Pi 5 (*estimate*). One unreviewed project reports 21.4 ms per embedding on a Pi 5, 1 thread, ONNX Runtime (https://huggingface.co/sraivante/TinyFaceNet-1M) |

- **Pi 5 CPU budget:** no source gives the load of our existing stack (SLAM, AMCL, policy) on the Pi 5 (03 G9). Run face work only in "searching" and "greeting" modes.
- **Accelerators (optional):**
  - **AI HAT+:** Hailo-8L 13 TOPS or Hailo-8 26 TOPS, "from $70" [RPI-AIHAT]. `arcface_mobilefacenet` runs at 1666 fps on a Hailo-8L, measured on an x86 host [HAILO-MZ]. Pi 5 numbers aren't published.
  - **AI HAT+ 2:** Hailo-10H, 40 TOPS INT4 / 20 TOPS INT8, 8 GB RAM, US$130 [RPI-AIHAT].
  - **Software:** `hailo-apps` (MIT) supports Hailo-8/8L/10H on the Pi 5 [HAILO-APPS]. A SCRFD-10G + ArcFace-MobileFaceNet face-recognition demo exists (Seeed wiki). One forum report shows a hailort hang with the face model, unresolved.
  - **Verdict:** not needed if the GB10 does the embedding. Consider it only for offline operation.

## 4. Open-set identification

- **Metrics:** use NIST's definitions [NIST-1N].
  - **FPIR:** the share of *non-enrolled* searches that return any candidate above the threshold.
  - **FNIR:** the share of enrolled searches that miss.
  - NIST reports FNIR at FPIR = 0.003, with the threshold set per algorithm.
  - Our requirement ("low false-match rate") is a **target FPIR**.
- **Gallery size:** false positives scale with gallery size N [IF-GUIDE]. With independent comparisons, FPIR ≈ N · FMR (*derived*). With 50 people, an FMR of 1e-4 gives an FPIR of ≈0.5 % per search.
- **Threshold:**
  - Typical 1:1 cosine thresholds are 0.30–0.45 at FMR 1e-4…1e-5, but "always recompute" per model and population [IF-GUIDE].
  - Pick the threshold on a validation split, never on the test set.
  - Estimating an FMR needs ≥10 / FMR non-mate comparisons [IF-GUIDE], i.e. **10⁵ for 1e-4**.
- **Calibrating on our people (proposal):**
  1. Record each enrolled person with the car's own camera at 2–4 m, in several sessions.
  2. Mated scores come from these recordings.
  3. Non-mated scores come from cross-person pairs plus a licensed public set of unenrolled faces.
  4. Choose two thresholds: a **strict** one for actions that address someone by name (greeting, "hungry"), and a looser one for "candidate, go closer" during a search.
  5. Also require agreement across several frames of the same track.
  6. Recompute after any model change [IF-GUIDE].
- **Enrolment:**
  - **Use:** bright, sharp, full head and shoulders, flat lighting, neutral expression [AWS-CAM].
  - **Pose limits:** yaw within ±45°, pitch within 30° down or 45° up [AWS-CAM].
  - **No source** gives a number of images per person. Proposal (*estimate*): 10–30 quality-filtered frames per person, from the robot's camera at the working distance, with and without glasses, at 2–3 times of day. Plus a few near-frontal photos.
  - Store per-image embeddings and a quality-weighted mean. IJB-B/C are template-based (sets of images) benchmarks, which is why template pooling is standard.
- **Updating the gallery:** no source found. Proposal: add only embeddings matched far above the strict threshold, keep a cap per person, and log every addition so a wrong one can be removed. The risk is template drift or poisoning (*unverified*).

## 5. When the face isn't visible

- **Person re-ID (appearance and clothing).**
  - OSNet (torchreid, MIT): `osnet_x1_0` has 2.2 M params and 0.98 GFLOPs, with Market-1501 rank-1 94.2 % / mAP 82.6 % in-domain [REID].
  - Cross-domain is much weaker: `osnet_ain_x1_0` MSMT17→Market rank-1 70.1 % / mAP 43.3 % [REID]. Expect lower in our office until fine-tuned.
  - **Combine it with face ID:** when a face match is confident, attach the identity to that person's body track and same-day appearance embedding. Re-ID then holds the identity while the person turns away or walks off. Expire appearance data daily, because clothing changes.
- **VLM matching on the GB10.** On a Market-1501 subset, GPT-4o, Gemini 2.0 Flash, Claude 3.5 Sonnet and Qwen-VL-Max scored **41–54 %** accuracy on same/different-person pairs. Precision was 37–45 %, and the authors call them "not yet reliable for ReID" [LVLM-REID].
  - **Use VLMs for:** descriptions ("person in a red jacket by the window"), for search hints and for the mission layer.
  - **Don't use them for:** identity decisions.

## 6. Privacy and legal basics (high level, not legal advice)

- **Israel:**
  - **Amendment 13** to the Privacy Protection Law took effect on **2025-08-14**. It makes biometric identifiers "especially sensitive data" [IL-PPL].
  - **Consent** must be informed, purpose-limited and proportionate. Data subjects can inspect and correct their data [IL-PPL].
  - **Security:** the Data Security Regulations (2017) apply [IL-PPL].
  - **Workplace biometrics:** the PPA's 2024 guideline asks for notice, consent, proportionality, transparency, purpose limitation, security and data minimisation [IL-PPL].
  - The notification threshold (>100,000 data subjects) doesn't apply to us [IL-PPL].
  - We found **no** personal or household exemption in these sources.
- **GDPR-style reference:** "biometric data for the purpose of uniquely identifying a natural person" is a special category (Art. 9(1)); explicit consent for specified purposes lifts the prohibition (Art. 9(2)(a)) [GDPR].
  - The **EU AI Act** bans emotion inference in the workplace (Art. 5(1)(f)), and untargeted scraping to build face databases (Art. 5(1)(e)) [AIA].
  - "Funny greetings" must not be based on inferred emotions if we ever operate under EU rules.
- **Practical rules (fit plan §13.3):**
  - Written opt-in per person, stating the purpose (greet, deliver messages, owner check).
  - Store **embeddings only** on the GB10, encrypted. Delete raw enrolment images after embedding, or keep them encrypted for re-enrolment when the model changes.
  - Don't store probes of unknown people beyond the current track.
  - Delete on request, and keep an access log.
  - Nothing goes to the cloud.

## 7. Recommended pipeline for us

| # | Component | Runs on | Choice | Constraint |
|---|---|---|---|---|
| 1 | Find a person | Pi | Stock YOLO person detection + depth for distance | Steer to put the person 2–4 m ahead (§2.2) |
| 2 | Stop and look | Car behaviour | Stop for ≈1 s (*estimate*) before capture | No motion blur [AWS-CAM] |
| 3 | Capture | Pi | **Now:** HP60C at 1920×1080 if the driver allows (§2.2 USB caveat). **Better:** add a Pi Camera Module 3 ($25 [RPI-CAM]) tilted up to put eye height at 2–4 m in frame; 12 MP stills | Eye distance ≥≈48 px goal [IF-GUIDE]; HP60C 1080p gives ≈18–35 px at 2–4 m, the CM3 still ≈56–112 px (*derived*) |
| 4 | Face detection | Pi | YuNet (MIT) on a downscaled frame, then crop from full resolution | ≈6 ms per 160×120 frame on a Pi 4 [OCV-BENCH]; Pi 5 to be measured |
| 5 | Send | Wi-Fi | Face crop (≈2× face box) + timestamp, car ID, bbox, landmarks, depth distance, track ID | Kilobytes per crop; no video stream |
| 6 | Align + quality + embed | GB10 | **Accuracy-first:** CVLFace/AdaFace (IR101 or ViT KP-RPE; feature norm = quality). **Licence-clean:** SFace. Run both on our data and pick | Licence decision (§1) |
| 7 | Template + open-set match | GB10 | Quality-weighted mean over the track; strict and loose thresholds; k-of-n frames | Calibrated per §4 |
| 8 | Hold identity | GB10 | OSNet-AIN body embedding linked to the face ID, same day only | Cross-domain accuracy is low [REID] |
| 9 | Describe | GB10 | Local VLM, for descriptions only | Not for identity [LVLM-REID] |
| 10 | Offline fallback | Pi | YuNet + SFace on CPU; optional Hailo HAT | ≈23–35 ms per face (*estimate*) |

**What to measure first on the real car**
1. **Geometry:**
   - Is the HP60C RGB at 640×480 a scale or a crop? Read `camera_info` (fx, fy).
   - At what distance do standing and seated colleagues' eyes enter the frame?
   - Does the `ascamera` driver deliver 1920×1080 RGB, at what fps, and what happens to depth?
2. **Eye distance in pixels at 1–4 m** for each stream (and for a CM3 if bought), compared with the §2.2 table.
3. **Detector recall vs distance:** stock MediaPipe short-range vs full-range vs YuNet.
4. **Score distributions** on 3–5 consenting colleagues at 2/3/4 m, standing and seated, under office lights: FNIR at a target FPIR, per model (AdaFace vs SFace vs buffalo_l, for evaluation only).
5. **Latency and CPU** on the Pi 5 while the driving stack runs; crop transfer and GB10 inference time.

---

## Findings that affect the plan

| # | Finding | Affects | What it means |
|---|---|---|---|
| H1 | **From 0.12 m with no tilt, standing faces are out of frame until ≈2.9–3.3 m** (*derived*) | Plan §13.1, open-issues #13, #17 | Plan the "approach to a polite distance" around 2–4 m recognition, not 1 m. Or add an upward camera or a mast |
| H2 | **The pitch limit (≈30°) also forces ≥≈2.4 m (standing) from this height** | #13, #17 | Recognition is a 2–4 m task: you need high-res pixels, not a closer approach |
| H3 | **The stock 640×480 stream gives ≈6–10 px eye distance where faces are visible** | 03 §1.2, #13 | Need 1080p from the HP60C (USB 2.0 limit unclear) or a second camera. Super-resolution is not a fix |
| H4 | **Hiwonder's face detector is MediaPipe short-range (≤2 m)** | Plan §13.2 table | Replace it with YuNet or MediaPipe full-range for our distances |
| H5 | **The best open low-res models' weights are research-only** (InsightFace explicitly; AdaFace via training data) | #13 "check its license" | User/legal decision before choosing; SFace/dlib are the clean fallbacks |
| H6 | **Detection on the Pi, embedding and matching on the GB10, crops only over Wi-Fi** | Plan §13.2, #14 | Shared gallery for both cars; the Pi load stays small; a CPU fallback exists |
| H7 | **VLMs are not reliable for person identity** (41–54 % pair accuracy) | #14 | Use the VLM for descriptions; identity comes from face + re-ID |
| H8 | **Face templates are "especially sensitive" under Israeli law since 2025-08-14** | Plan §13.3 | Written consent, purpose limits, encryption, deletion, embeddings only |

## Open items

- **Measure on the car:**
  - HP60C RGB intrinsics at 640×480 (`camera_info`); camera height and tilt (03 §1.2).
  - Whether `ascamera` supports 1920×1080 or 1280×720 RGB, at what fps, over USB 2.0, with depth on.
  - Eye distance in pixels vs distance for each stream; where colleagues' faces enter the frame (standing and seated).
  - Detector recall vs distance: MediaPipe short-range vs full-range vs YuNet.
  - Pi 5 latency and CPU for YuNet and SFace with the driving stack running; crop round-trip time to the GB10.
  - Score distributions and FNIR at a target FPIR for AdaFace / SFace / buffalo_l on consenting colleagues at 2–4 m, in office lighting.
- **Decide (user):**
  - Model licence route (§1).
  - Whether to add a Pi Camera Module 3 (upward tilt, CSI) or a camera mast.
  - Consent form and retention policy.
- **Check further:**
  - Whether the weights in Hailo's `arcface_mobilefacenet` come from InsightFace.
  - The WebFace260M licence text (face-benchmark.org; not read).
  - The full NIST IR 8173 findings on camera placement (PDF too large to fetch).
  - Whether ONNX Runtime GPU / TensorRT builds exist for aarch64 + CUDA 13 on the GB10 for these models.
- **No source found for:** images per person for enrolment; safe gallery-update rules; how a low camera's backlight affects accuracy; GB10 inference speed for these models.
