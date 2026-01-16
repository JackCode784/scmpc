// #include <bits/stdc++.h>

// Function to get determinant of matrix
int determinantOfMatrix(int** mat, int n)
{
    // Initialize result
    int num1, num2, det = 1, index,
                    total = 1; 

    // Temporary array for storing row
    int temp[n + 1];

    // Loop for traversing the diagonal elements
    for (int i = 0; i < n; i++) 
    {
        // Initialize the index
        index = i; 

        // Finding the index which has 
        // non zero value
        while (index < n && mat[index][i] == 0) 
        {
            index++;
        }

        // if there is non zero element
        if (index == n) 
        {
            // the determinant of matrix 
            // as zero
            continue;
        }
        if (index != i) 
        {
            // Loop for swapping the diagonal 
            // element row and index row
            for (int j = 0; j < n; j++) 
            {
                swap(mat[index][j], mat[i][j]);
            }

            // Determinant sign changes when we 
            // shift rows go through determinant 
            // properties
            det = det * pow(-1, index - i);
        }

        // Storing the values of diagonal 
        // row elements
        for (int j = 0; j < n; j++) 
        {
            temp[j] = mat[i][j];
        }

        // Traversing every row below the
        // diagonal element
        for (int j = i + 1; j < n; j++) 
        {
            // Value of diagonal element
            num1 = temp[i]; 

            // Value of next row element
            num2 = mat[j][i]; 

            // Traversing every column of row
            // and multiplying to every row
            for (int k = 0; k < n; k++) 
            {
                // Multiplying to make the diagonal
                // element and next row element equal
                mat[j][k]
                    = (num1 * mat[j][k]) - (num2 * temp[k]);
            }
            total = total * num1; // Det(kA)=kDet(A);
        }
    }

    // Multiplying the diagonal elements to 
    // get determinant
    for (int i = 0; i < n; i++) 
    {
        det = det * mat[i][i];
    }

    // Det(kA)/k=Det(A);
    return (det / total); 
}
