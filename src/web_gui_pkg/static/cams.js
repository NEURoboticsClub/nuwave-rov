// Minimal cameras-only view loaded with ?cams: one large primary camera,
// the other three stacked on the right
// Click a camera to make it the primary camera

(function () {
    const CAMERA_COUNT = 4;
    let primaryId = 0;
    const cameras = new Map();

    document.getElementById("dashboard")?.remove();

    const root = document.createElement("div");
    root.className = "cams";

    const primarySlot = document.createElement("div");
    primarySlot.className = "cams__primary";

    const sideSlot = document.createElement("div");
    sideSlot.className = "cams__side";

    root.append(primarySlot, sideSlot);
    document.body.appendChild(root);

    for (let id = 0; id < CAMERA_COUNT; id++) {
        const card = document.createElement("div");
        card.className = "cam";

        const canvas = document.createElement("canvas");
        canvas.width = 640;
        canvas.height = 480;

        const label = document.createElement("div");
        label.className = "cam__label";
        label.textContent = `CAM ${id}`;

        card.append(canvas, label);
        card.addEventListener("click", () => setPrimary(id));

        cameras.set(id, { card, canvas, pendingFrame: null, decoding: false });
    }

    function setPrimary(id) {
        primaryId = id;
        primarySlot.innerHTML = "";
        sideSlot.innerHTML = "";
        for (let i = 0; i < CAMERA_COUNT; i++) {
            const cam = cameras.get(i);
            const isPrimary = i === primaryId;
            cam.card.classList.toggle("cam--primary", isPrimary);
            (isPrimary ? primarySlot : sideSlot).appendChild(cam.card);
        }
    }

    setPrimary(primaryId);

    function updateCamera(id, bytes) {
        const cam = cameras.get(id);
        if (!cam) return;
        cam.pendingFrame = bytes;
        if (!cam.decoding) decode(cam, id);
    }

    function decode(cam, id) {
        const next = cam.pendingFrame;
        if (!next) {
            cam.decoding = false;
            return;
        }
        cam.pendingFrame = null;
        cam.decoding = true;

        const blob = new Blob([next], { type: "image/jpeg" });
        createImageBitmap(blob).then((bitmap) => {
            const ctx = cam.canvas.getContext("2d");
            cam.canvas.width = bitmap.width;
            cam.canvas.height = bitmap.height;
            ctx.drawImage(bitmap, 0, 0);
            bitmap.close();
        }).catch(() => {
            console.error(`Failed to decode image for camera ${id}`);
        }).finally(() => decode(cam, id));
    }

    // Binary frames only: byte 0 is the camera id
    const ws = new WebSocket(`ws://${location.host}/ws`);
    ws.binaryType = "arraybuffer";
    ws.onclose = () => setTimeout(() => location.reload(), 2000);
    ws.onmessage = (event) => {
        if (typeof event.data === "string") return;
        const buf = new Uint8Array(event.data);
        updateCamera(buf[0], buf.subarray(1));
    };
})();
