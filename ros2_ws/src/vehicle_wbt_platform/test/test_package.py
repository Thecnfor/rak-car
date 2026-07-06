def test_package_imports():
    import vehicle_wbt_platform
    assert vehicle_wbt_platform.__version__ == "0.1.0"


def test_main_module_imports():
    from vehicle_wbt_platform import __main__  # noqa: F401
