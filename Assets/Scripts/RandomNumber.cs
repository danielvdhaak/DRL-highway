﻿/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A pseudo-random number generator (PRNG).
/// </summary>
public class RandomNumber
{
    /// <summary>
    /// Returns a random uniformly distributed float number on the interval [0, 1], i.e. including 0 and 1.
    /// </summary>
    public float Uniform()
    {
        return Random.Range(0.0f, 1.0f);
    }

    /// <summary>
    /// Returns a random uniformly distributed float number on the interval [0, max], i.e. including 0 and max.
    /// </summary>
    public float Uniform(float max)
    {
        return Random.Range(0.0f, max);
    }

    /// <summary>
    /// Returns a random uniformly distributed float number on the interval [min, max], i.e. including min and max.
    /// </summary>
    public float Uniform(float min, float max)
    {
        return Random.Range(min, max);
    }

    /// <summary>
    /// Returns a random standard normally distributed float number.
    /// The number is generated by the Box-Muller method.
    /// </summary>
    public float Gaussian()
    {
        float U1 = Uniform(0.001f, 1.0f);
        float U2 = Uniform(0.001f, 1.0f);

        return Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Cos(2.0f * Mathf.PI * U2);
    }

    /// <summary>
    /// Returns a random normally distributed float number with a mean and standard deviation std.
    /// The number is generated by the Box-Muller method.
    /// </summary>
    public float Gaussian(float mean, float std)
    {
        float U1 = Uniform(0.001f, 1.0f);
        float U2 = Uniform(0.001f, 1.0f);

        return mean + std *  Mathf.Sqrt(-2.0f * Mathf.Log(U1)) * Mathf.Cos(2.0f * Mathf.PI * U2);
    }

    /// <summary>
    /// Returns a random negative shifted exponentially distributed float number with a mean and minimum.
    /// </summary>
    public float NegativeExponential(float mean, float min)
    {
        float R = Uniform(0.001f, 1.0f);

        return (mean - min) * (-Mathf.Log(R)) + mean - min;
    }

    /// <summary>
    /// Return a random float number on the interval [0, 1] generated by a 2D Perlin noise.
    /// </summary>
    public float Perlin()
    {
        return Mathf.PerlinNoise(Uniform(), Uniform());
    }
}
