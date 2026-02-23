import math

def _format_bool_to_str(v):
    if v is None:
        return "[dim]—[/dim]"
    return "[green]✓[/green]" if v else "[red]✗[/red]"


def _format_str_to_str(v):
    if v is None:
        return "[dim]—[/dim]"
    s = str(v)
    return s if s else "[dim]—[/dim]"


def _format_float_to_str(v, ndigits=2, multiplier=1):
    if v is None:
        return "[dim]—[/dim]"
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return "NaN"
    if multiplier == 1:
        return f"{v:.{ndigits}f}"
    else:
        return f"{v*multiplier:.{ndigits}f}"


def _stamp_to_clock_str(sec: int, nanosec: int) -> str:
    try:
        import datetime as _dt
        dt = _dt.datetime.fromtimestamp(sec)
        return dt.strftime("%H:%M:%S")
    except Exception:
        return str(sec)
    
def _radians_to_degree(radians):
    return radians * 180.0 / math.pi

def _wrap_degrees(d):
    return (d + 360) % 360