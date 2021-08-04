 
#ifndef __DIGAME_MATH_H__
#define __DIGAME_MATH_H__

//************************************************************************
// Math functions
//************************************************************************

//************************************************************************
float mean(float a[], int numSamples){
    float result;
    
    result = 0;   
    for (int i=0; i<numSamples; i++) {
      result += a[i];
    }
    result /= numSamples;
    return result;  
}


//************************************************************************
// Calculate the correlation coefficient between two arrays
float correlation(float x[], float y[], int numSamples){ 
    float sx, sy, sxy, denom; 
    float mx, my; // means
    float r; // correlation coefficient

    /* Calculate the means */
    mx = mean(x, numSamples);
    my = mean(y, numSamples);
    
    /* Calculate the denominator */
    sx = 0;
    sy = 0;
    
    for (int i=0; i<numSamples; i++) {
      sx += (x[i] - mx) * (x[i] - mx);
      sy += (y[i] - my) * (y[i] - my);
    }
    
    denom = sqrt(sx*sy);

   /* Calculate the correlation coefficient */
    sxy = 0;
    for (int i=0; i<numSamples; i++) {
          sxy += (x[i] - mx) * (y[i] - my); 
    }
    r = sxy / denom;
    return r;
}

#endif
