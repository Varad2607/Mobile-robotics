def compute_fibonacci(n):
    """Return the nth Fibonacci number.

    >>> compute_fibonacci(0)
    0
    >>> compute_fibonacci(1)
    1
    >>> compute_fibonacci(2)  # 0 + 1
    1
    >>> compute_fibonacci(3)  # 1 + 1
    2
    >>> compute_fibonacci(4)  # 1 + 2
    3
    """
    # BEGIN QUESTION 1.1
    if n == 0:
        return 0
    elif n == 1 or n==2:
        return 1

    previous_fib, current_fib = 1, 1
    for _ in range(3, n + 1):
        previous_fib, current_fib = current_fib, previous_fib + current_fib

    return current_fib
    
    # END QUESTION 1.1
