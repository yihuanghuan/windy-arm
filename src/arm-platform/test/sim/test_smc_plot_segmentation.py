import numpy as np

from task_space_smc.sim.plot_whole_task_smc import _drawing_segments


def test_drawing_segments_split_on_pen_up_and_stroke_change():
    points = np.asarray([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [9.0, 9.0, 9.0],
        [2.0, 0.0, 0.0],
        [3.0, 0.0, 0.0],
        [4.0, 0.0, 0.0],
        [5.0, 0.0, 0.0],
    ])
    pen_down = np.asarray([1, 1, 0, 1, 1, 1, 1])
    stroke_ids = np.asarray([0, 0, -1, 1, 1, 2, 2])

    segments = _drawing_segments(points, pen_down, stroke_ids)

    assert [len(segment) for segment in segments] == [2, 2, 2]
    assert all(not np.any(np.all(segment == [9.0, 9.0, 9.0], axis=1))
               for segment in segments)
