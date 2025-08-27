import React, { useEffect, useRef, useState } from "react";
import axios from "axios";
import "../TableMapping.css";

type DetectionBox = {
  id: string;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
};

export default function TableMapping() {
  const [imageUrl, setImageUrl] = useState("");
  const [boxes, setBoxes] = useState<DetectionBox[]>([]);
  const [selectedBox, setSelectedBox] = useState<DetectionBox | null>(null);
  const [tableIdInput, setTableIdInput] = useState("");

  const [showMappingImage, setShowMappingImage] = useState(true);
  const [logs, setLogs] = useState<string[]>([]);

  // 스크롤 영역 ref (맨 아래 고정, 사용자가 스크롤 올리면 유지)
  const scrollRef = useRef<HTMLDivElement | null>(null);

  // YOLO bbox/이미지 폴링(5초)
  useEffect(() => {
    let stopped = false;

    const fetchDetections = async () => {
      try {
        const res = await axios.get("http://localhost:3000/get_detections", {
          params: { t: Date.now() },
          headers: { "Cache-Control": "no-cache" },
        });
        if (stopped) return;
        setBoxes(res.data.detections ?? []);
        setImageUrl("http://localhost:3000" + (res.data.image_url ?? ""));
      } catch (err) {
        if (!stopped) console.error("감지 결과 가져오기 실패:", err);
      }
    };

    fetchDetections();
    const interval = setInterval(fetchDetections, 5000);
    return () => {
      stopped = true;
      clearInterval(interval);
    };
  }, []);

  // 서버 로그 폴링(5초) — 전체 로그 받아서 스크롤로 보기
  useEffect(() => {
    let stopped = false;

    const fetchLogs = async () => {
      try {
        const res = await axios.get("http://localhost:3000/logs", {
          params: { t: Date.now() },
          headers: { "Cache-Control": "no-cache" },
        });
        if (stopped) return;
        const serverLogs: string[] = res.data?.logs ?? [];
        setLogs(serverLogs); // ← 자르지 않고 전부 보관 (스크롤로 조회)
      } catch (e) {
        if (!stopped) console.error("로그 가져오기 실패:", e);
      }
    };

    fetchLogs();
    const id = setInterval(fetchLogs, 5000);
    return () => {
      stopped = true;
      clearInterval(id);
    };
  }, []);

  // 새 로그가 들어왔을 때, 사용자가 거의 맨 아래에 있었으면 자동으로 맨 아래로 이동
  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    const nearBottom = el.scrollHeight - (el.scrollTop + el.clientHeight) < 40;
    if (nearBottom) {
      el.scrollTop = el.scrollHeight;
    }
  }, [logs]);

  const handleBoxClick = (box: DetectionBox) => {
    setSelectedBox(box);
    setTableIdInput("");
  };

  const handleSubmit = async () => {
    if (!selectedBox || !tableIdInput) return;
    try {
      await axios.post("http://localhost:3000/set_table_id", {
        box_id: selectedBox.id,
        table_id: Number(tableIdInput),
      });
      alert(`테이블 ${tableIdInput} 등록 완료!`);
      setSelectedBox(null);
      setTableIdInput("");
    } catch (err: any) {
      console.error("등록 실패:", err.response?.data || err.message);
      alert("등록 실패: " + (err.response?.data?.error || "서버 오류"));
    }
  };

  return (
    <div className="container">
      {/* 상단 로고 + 타이틀 */}
      <div className="title-bar">
        <img src="/logoTable.png" alt="로고" className="side-logo" />
        <h1 className="page-title">테이블 번호 매핑 시스템</h1>
        <img src="/logoTable.png" alt="로고" className="side-logo" />
      </div>

      {/* 본문: 2열 고정 그리드 */}
      <div className="stream-and-image">
        {/* 왼쪽: 라이브 스트림 */}
        <div className="video-stream" style={{ width: 640 }}>
          <h3 className="section-title">Live Camera Stream</h3>
          <img
            src="http://localhost:3000/video_feed"
            onError={(e) => {
              e.currentTarget.src = "/placeholder.png";
            }}
            alt="Live Camera"
            className="live-video"
            width={640}
            height={480}
          />
        </div>

        {/* 오른쪽: Mapping */}
        <div className="mapping-column" style={{ width: 640 }}>
          {/* 제목 + 버튼 줄 */}
          <div
            style={{
              display: "flex",
              alignItems: "center",
              justifyContent: "space-between",
              marginBottom: 8,
              gap: 12,
            }}
          >
            <h3 className="section-title" style={{ margin: 0 }}>
              Mapping Table
            </h3>

            <div style={{ display: "flex", gap: 8 }}>
              <button
                type="button"
                onClick={() => setShowMappingImage(true)}
                className="mapping-done-btn"
                disabled={showMappingImage}
                title={showMappingImage ? "이미 표시 중" : "숨긴 이미지를 다시 보기"}
              >
                다시 보기
              </button>
              <button
                type="button"
                onClick={() => {
                  setShowMappingImage(false); // 이미지 숨기고 로그 뷰
                  setSelectedBox(null);
                }}
                className="mapping-done-btn"
                disabled={!showMappingImage}
                title="이미지 숨기기"
              >
                매핑 완료
              </button>
            </div>
          </div>

          {/* 고정 크기 컨테이너 */}
          <div className="image-container">
            {showMappingImage ? (
              <>
                <img
                  src={imageUrl}
                  alt="YOLO detection"
                  style={{
                    width: "100%",
                    height: "100%",
                    objectFit: "contain",
                    display: "block",
                  }}
                />
                {boxes.map((box) => (
                  <div
                    key={box.id}
                    className="bbox"
                    style={{
                      left: box.x1,
                      top: box.y1,
                      width: box.x2 - box.x1,
                      height: box.y2 - box.y1,
                      border:
                        selectedBox?.id === box.id
                          ? "3px solid #ff9800"
                          : "2px solid #70C1B3",
                      boxSizing: "border-box",
                      pointerEvents: "auto",
                    }}
                    onClick={() => handleBoxClick(box)}
                  >
                    {box.id}
                    {selectedBox?.id === box.id && (
                      <div className="inline-modal">
                        <p>이 테이블에<br />번호를 지정해 주세요.</p>
                        <input
                          type="number"
                          placeholder="테이블 번호"
                          value={tableIdInput}
                          onChange={(e) => setTableIdInput(e.target.value)}
                          step={1}
                          min={1}
                        />
                        <button onClick={handleSubmit}>등록</button>
                      </div>
                    )}
                  </div>
                ))}
              </>
            ) : (
              // 로그 출력 — 좌상단 붙임 + 전체 높이 스크롤
              <div className="image-placeholder log-panel">
                {logs.length > 0 ? (
                  <div className="log-scroll" ref={scrollRef}>
                    <div className="log-list">
                      {logs.map((line, idx) => (
                        <div key={idx} className="log-line">
                          {line}
                        </div>
                      ))}
                    </div>
                  </div>
                ) : (
                  <p className="log-empty">로그가 아직 없습니다.</p>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}