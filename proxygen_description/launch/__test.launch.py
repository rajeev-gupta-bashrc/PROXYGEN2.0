from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    x = LaunchConfiguration('x')
    
    # Create conditional expression
    value_to_print = PythonExpression([
        'str(float(', x, ')) if float(', x, ') < 1 else str(1 - float(', x, '))'
    ])

    return LaunchDescription([
        LogInfo(msg=['Result: ', value_to_print])
    ])