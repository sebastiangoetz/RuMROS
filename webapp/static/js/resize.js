function makeResizablePanels({ containerId, leftId, rightId, dividerId }) {
  const container = document.getElementById(containerId);
  const leftPanel = document.getElementById(leftId);
  const rightPanel = document.getElementById(rightId);
  const divider = document.getElementById(dividerId);

  let isDragging = false;

  divider.addEventListener("mousedown", (e) => {
    isDragging = true;

    document.body.style.userSelect = "none";
    document.body.style.pointerEvents = "none";

    document.addEventListener("mousemove", handleMouseMove);
    document.addEventListener("mouseup", () => {
      isDragging = false;
      document.body.style.userSelect = "";
      document.body.style.pointerEvents = "";
      document.removeEventListener("mousemove", handleMouseMove);
    });

    e.preventDefault();
  });

  function handleMouseMove(e) {
    if (!isDragging) return;

    const offsetLeft = container.offsetLeft;
    const containerWidth = container.clientWidth;

    let newLeftWidth = e.clientX - offsetLeft;

    const minLeftWidth = 0;
    const minRightWidth = 200;
    const maxLeftWidth = containerWidth - minRightWidth;

    if (newLeftWidth < minLeftWidth) newLeftWidth = minLeftWidth;
    if (newLeftWidth > maxLeftWidth) newLeftWidth = maxLeftWidth;

    leftPanel.style.flexBasis = `${newLeftWidth}px`;
    rightPanel.style.flexBasis = `${containerWidth - newLeftWidth}px`;

    if (__ARROWS__)
      __ARROWS__.forEach(a => a.position());
  }
}
