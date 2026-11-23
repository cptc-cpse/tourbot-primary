import colorsys
import threading
import tkinter as tk
from queue import Empty, Queue

from serial.serialutil import SerialException

from communication.mqtt_client import Client
from communication.message import Message
import services.lidar.lidar_data_collection as lidar
import services.oak.oak_data_collection as oak
import services.mapping.mapping as mapping

CELL_SIZE = 12
BACKGROUND_COLOR = "#111827"
CELL_COLORS = {
    -1: "#6b7280",
    1: "#22c55e",
    2: "#ef4444",
    3: "#f59e0b",
}
CELL_NAMES = {
    -1: "Unknown",
    1: "Robot",
    2: "Stationary obstacle",
    3: "Moving obstacle",
}


def cell_color(value: int) -> str | None:
    """Return no color for free cells and a stable color for every other value."""
    if value == 0:
        return None
    if value in CELL_COLORS:
        return CELL_COLORS[value]

    hue = (value * 0.618033988749895) % 1.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.68, 0.95)
    return f"#{round(red * 255):02x}{round(green * 255):02x}{round(blue * 255):02x}"


def validate_grid(payload: object) -> list[list[int]]:
    if not isinstance(payload, dict):
        raise ValueError("Map message payload must be an object")

    grid_value = payload.get("map")
    if not isinstance(grid_value, list) or not grid_value:
        raise ValueError("Map message must contain a non-empty grid")

    grid: list[list[int]] = []
    expected_width: int | None = None
    for row in grid_value:
        if not isinstance(row, list):
            raise ValueError("Map grid rows must be lists")
        if expected_width is None:
            expected_width = len(row)
            if expected_width == 0:
                raise ValueError("Map grid rows must not be empty")
        elif len(row) != expected_width:
            raise ValueError("Map grid rows must have equal widths")

        validated_row: list[int] = []
        for value in row:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError("Map grid cells must be integers")
            validated_row.append(value)
        grid.append(validated_row)

    return grid


class MapVisualizer:
    def __init__(self, client: Client):
        self.root = tk.Tk()
        self.root.title("Occupancy Map")
        self.root.geometry("900x700")
        self.root.configure(background=BACKGROUND_COLOR)
        self._closed = False
        self._pending_grids: Queue[list[list[int]]] = Queue()
        self._latest_grid: list[list[int]] | None = None
        self._resize_pending = False

        self.legend = tk.Frame(self.root, background=BACKGROUND_COLOR)
        self.legend.pack(fill="x", padx=8, pady=8)

        frame = tk.Frame(self.root)
        frame.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(
            frame,
            background=BACKGROUND_COLOR,
            highlightthickness=0,
        )
        horizontal_scrollbar = tk.Scrollbar(
            frame,
            orient="horizontal",
            command=self.canvas.xview,
        )
        vertical_scrollbar = tk.Scrollbar(
            frame,
            orient="vertical",
            command=self.canvas.yview,
        )
        self.canvas.configure(
            xscrollcommand=horizontal_scrollbar.set,
            yscrollcommand=vertical_scrollbar.set,
        )
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        vertical_scrollbar.grid(row=0, column=1, sticky="ns")
        horizontal_scrollbar.grid(row=1, column=0, sticky="ew")
        frame.grid_rowconfigure(0, weight=1)
        frame.grid_columnconfigure(0, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self._close)
        client.subscribe(mapping.MAP_TOPIC, self._on_map_message)
        self.root.after(50, self._drain_grid_queue)

    def run(self) -> None:
        self.root.mainloop()

    def _on_map_message(self, message: Message) -> None:
        self._pending_grids.put(validate_grid(message.payload))

    def _drain_grid_queue(self) -> None:
        grid: list[list[int]] | None = None
        while True:
            try:
                grid = self._pending_grids.get_nowait()
            except Empty:
                break

        if grid is not None:
            self._draw_grid(grid)
        if not self._closed:
            self.root.after(50, self._drain_grid_queue)

    def _draw_grid(self, grid: list[list[int]]) -> None:
        self._latest_grid = grid
        self._render_grid(grid)

    def _render_grid(self, grid: list[list[int]]) -> None:
        self.canvas.delete("all")
        height = len(grid)
        width = len(grid[0])
        canvas_width = max(1, self.canvas.winfo_width())
        canvas_height = max(1, self.canvas.winfo_height())
        cell_size = self._fit_cell_size(
            width,
            height,
            canvas_width,
            canvas_height,
        )
        map_width = width * cell_size
        map_height = height * cell_size
        offset_x = max(0.0, (canvas_width - map_width) / 2.0)
        offset_y = max(0.0, (canvas_height - map_height) / 2.0)
        self.canvas.configure(
            scrollregion=(
                0,
                0,
                max(canvas_width, map_width),
                max(canvas_height, map_height),
            ),
        )

        values = sorted({value for row in grid for value in row if value != 0})
        self._draw_legend(values)

        for row_index, row in enumerate(grid):
            for column_index, value in enumerate(row):
                color = cell_color(value)
                if color is None:
                    continue
                x0 = offset_x + column_index * cell_size
                y0 = offset_y + row_index * cell_size
                self.canvas.create_rectangle(
                    x0,
                    y0,
                    x0 + cell_size,
                    y0 + cell_size,
                    fill=color,
                    outline=color,
                )

    @staticmethod
    def _fit_cell_size(
        grid_width: int,
        grid_height: int,
        canvas_width: int,
        canvas_height: int,
    ) -> float:
        return min(
            CELL_SIZE,
            canvas_width / grid_width,
            canvas_height / grid_height,
        )

    def _on_canvas_configure(self, _event: tk.Event) -> None:
        if self._latest_grid is None or self._resize_pending:
            return
        self._resize_pending = True
        self.root.after_idle(self._redraw_latest_grid)

    def _redraw_latest_grid(self) -> None:
        self._resize_pending = False
        if self._latest_grid is not None and not self._closed:
            self._render_grid(self._latest_grid)

    def _draw_legend(self, values: list[int]) -> None:
        for child in self.legend.winfo_children():
            child.destroy()

        tk.Label(
            self.legend,
            text="0: Empty",
            background=BACKGROUND_COLOR,
            foreground="white",
        ).pack(side="left", padx=6)
        for value in values:
            color = cell_color(value)
            if color is None:
                continue
            label = CELL_NAMES.get(value, f"Value {value}")
            tk.Label(
                self.legend,
                text=f"{value}: {label}",
                background=color,
                foreground="black",
                padx=6,
                pady=3,
            ).pack(side="left", padx=4)

    def _close(self) -> None:
        self._closed = True
        self.root.destroy()


def main() -> None:
    client = Client(qos=0, retain=False, process_id="main")
    visualizer = MapVisualizer(client)

    oak_thread = threading.Thread(target=oak.stream_data)
    oak_thread.start()
    try:
        lidar_thread = threading.Thread(target=lidar.stream_data)
        lidar_thread.start()
    except SerialException:
        print("LiDAR not found")
    map_thread = threading.Thread(target=mapping.maintain_map)
    map_thread.start()

    visualizer.run()


if __name__ == "__main__":
    main()
