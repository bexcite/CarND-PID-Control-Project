# Describe the effect each of the P, I, D components had in your implementation.

Proportional part (P) adds the main control to the vehicle that tries to correct the CTE error by changing the steering angle. Without proportional part vehicle is almost not turning towards the error correction.

Differential part (D) removes oscillation by dumping overshooting caused by proportional (P) component. Without differential part automatically tuned proportional parameter cause huge oscillations that drive vehicle off the track.

Integral part (I) removes the bias or accumulated error. Without integral part vehicle moves slowly to the right side of the road.


# Describe how the final hyperparameters were chosen.

That was a long journey.
 
Firstly, I quickly and easily found parameters manually that worked reasonably well on throttle 0.5 and even 0.7 (though it's not considered safe driving on that speed). But I was unable to find the parameters that works good enough for speeds 0.75 or 0.8. So I've implemented a twiddle algorithm.
 
Twiddle algorithm helped tune the manually selected parameters to an optimal value on a given speed (throttle 0.6 and 0.7). But still twiddle tends to stuck in local optima and can't find good solutions without manual intervention in parameter selection.
  
So I've decided to run twiddle starting from a lower throttle (0.3), arbitrary selected parameters (1, 1, 1) and gradually increase throttle when sum of delta params is becoming lower then 0.0005. Though it didn't work well. See video below.
  
<< oscillator video >>
  
Twiddle worked as intended and drives the error down, which was defined as TotalError/(Dist * Dist). Factor over distance squared was used because car could crash at any time. Though the total error representations is not capturing the level of oscillations directly.

So I had to return to manually selected parameters and just optimized them using twiddle on throttle value starting from 0.6 up to 0.7. (i.e. local optimization in an already good enough region rather then global search over wide parameters space)
   
There was about 3000 tries in simulator (automatic tries) where I've been able to find the best parameters in the region of manually selected values.
 
Result is reasonably good on throttle value 0.7 and by decreasing it to 0.5 we have a safe driving experience. Here is the result for 0.7 throttle.

<< video of 0.7 throttle >>


