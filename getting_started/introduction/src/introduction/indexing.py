from introduction.fibonacci import compute_fibonacci
import numpy as np


def extract_fibonacci_rows(data):
    """Extract the rows of a numpy array that correspond to the Fibonacci numbers,
    using integer array indexing.

    >>> data = np.array([[ 4,  7,  8],
    ...                  [14, 17, 18],
    ...                  [24, 27, 28],
    ...                  [34, 37, 38]])
    >>> extract_fibonacci_rows(data)
    array([[ 4,  7,  8],
           [14, 17, 18],
           [14, 17, 18],
           [24, 27, 28],
           [34, 37, 38]])
    """
    # BEGIN QUESTION 3.1
    array=[]
    d=data.shape[0]
    for i in range(d):
       array.append(data[compute_fibonacci(i)])
       arr=np.array(array)
    return arr 
    
    # END QUESTION 3.1
   

def increment_rows_with_odd_first_element(data):
    """Add one to rows of a numpy array where the 0th column element is odd,
    using Boolean array indexing.

    NOTE: This function does not have a return value: your implementation
    should modify the data argument in-place.

    >>> data = np.array([[ 0,  1,  2],
    ...                  [ 3,  4,  5],
    ...                  [ 6,  7,  8],
    ...                  [ 9, 10, 11],
    ...                  [12, 13, 14],
    ...                  [15, 16, 17]])
    >>> increment_rows_with_odd_first_element(data)
    >>> data
    array([[ 0,  1,  2],
           [ 4,  5,  6],
           [ 6,  7,  8],
           [10, 11, 12],
           [12, 13, 14],
           [16, 17, 18]])
    """
    # BEGIN QUESTION 3.2
    
    
    # Create a Boolean mask for rows where the 0th column is odd
    odd_rows_mask = data[:, 0] % 2 == 1
    
    # Increment rows where the 0th column is odd
    data[odd_rows_mask] += 1
    # END QUESTION 3.2
    return None
