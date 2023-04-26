from ros2diagnostics_cli.api import open_file_for_output


def test_file_create():
    f = open_file_for_output("/tmp/1/a.csv")
    assert f is not None

test_file_create()