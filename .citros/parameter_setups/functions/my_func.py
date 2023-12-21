def my_awesome_func(num):
    """
    A few things to notice:
    1. Any imports must be done inside the function.
    2. Return either native types or numpy scalars. Do not return non-scalar numpy values.
    """
    import numpy

    return numpy.multiply(num, 2.0)

def func_with_context(num, citros_context):
    return num + float(citros_context['run_id'])*10