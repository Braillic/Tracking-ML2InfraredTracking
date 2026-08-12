using System;
using System.Collections.Generic;
using UnityEngine;

public class MarkerDetector : MonoBehaviour
{
    // Same settings as the Python detector
    public float rawMin = 5.0f;
    public float rawMax = 3000.0f;

    public float percentileFloor = 100.0f;
    public float percentile = 90.0f;

    public int markerMinArea = 2;
    public int markerMaxArea = 2000;
    public float markerMaxAspect = 4.0f;

    public int maxMarkers = 4;

    public struct DetectionResult
    {
        public List<Vector2Int> centers;
        public float cutoff;
    }

    public DetectionResult Detect(float[] depth, int width, int height)
    {
        byte[] intensity = ColouriseDepth(depth);

        float cutoff = CalculatePercentileCutoff(
            intensity,
            percentileFloor,
            percentile
        );

        bool[] mask = new bool[intensity.Length];

        for (int i = 0; i < intensity.Length; i++)
        {
            mask[i] = intensity[i] >= cutoff;
        }

        List<Candidate> candidates =
            FindConnectedComponents(mask, intensity, width, height);

        // Python:
        // candidates.sort(key=lambda item: item[0], reverse=True)
        candidates.Sort((a, b) =>
            b.meanIntensity.CompareTo(a.meanIntensity));

        List<Vector2Int> centers = new();

        int count = Math.Min(maxMarkers, candidates.Count);

        for (int i = 0; i < count; i++)
        {
            centers.Add(candidates[i].center);
        }

        return new DetectionResult
        {
            centers = centers,
            cutoff = cutoff
        };
    }


    // --------------------------------------------------
    // Python equivalent: colourise_depth(... unity-raw)
    // --------------------------------------------------

    private byte[] ColouriseDepth(float[] depth)
    {
        byte[] grey = new byte[depth.Length];

        float denominator = Mathf.Max(rawMax - rawMin, 0.000001f);

        for (int i = 0; i < depth.Length; i++)
        {
            float d = depth[i];

            if (float.IsNaN(d) || float.IsInfinity(d))
            {
                grey[i] = 0;
                continue;
            }

            float normalized = (d - rawMin) / denominator;
            normalized = Mathf.Clamp01(normalized);

            // Same linear -> sRGB conversion as Python
            if (normalized <= 0.0031308f)
            {
                normalized *= 12.92f;
            }
            else
            {
                normalized =
                    1.055f * Mathf.Pow(normalized, 1.0f / 2.4f)
                    - 0.055f;
            }

            grey[i] = (byte)Mathf.RoundToInt(normalized * 255.0f);
        }

        return grey;
    }


    // --------------------------------------------------
    // Python equivalent:
    // max(floor, np.percentile(intensity, percentile))
    // --------------------------------------------------

    private float CalculatePercentileCutoff(
        byte[] intensity,
        float floor,
        float percentileValue)
    {
        if (intensity.Length == 0)
            return floor;

        byte[] sorted = (byte[])intensity.Clone();
        Array.Sort(sorted);

        float position =
            (sorted.Length - 1) * (percentileValue / 100.0f);

        int lower = Mathf.FloorToInt(position);
        int upper = Mathf.CeilToInt(position);

        float fraction = position - lower;

        float value =
            sorted[lower] +
            (sorted[upper] - sorted[lower]) * fraction;

        return Mathf.Max(floor, value);
    }


    private struct Candidate
    {
        public float meanIntensity;
        public Vector2Int center;
    }


    // --------------------------------------------------
    // Python equivalent:
    // cv2.connectedComponentsWithStats(... connectivity=8)
    // --------------------------------------------------

    private List<Candidate> FindConnectedComponents(
        bool[] mask,
        byte[] intensity,
        int width,
        int height)
    {
        bool[] visited = new bool[mask.Length];

        List<Candidate> candidates = new();

        int[] dx = { -1, 0, 1, -1, 1, -1, 0, 1 };
        int[] dy = { -1, -1, -1, 0, 0, 1, 1, 1 };

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = y * width + x;

                if (!mask[index] || visited[index])
                    continue;

                Queue<Vector2Int> queue = new();
                queue.Enqueue(new Vector2Int(x, y));
                visited[index] = true;

                int area = 0;

                int minX = x;
                int maxX = x;
                int minY = y;
                int maxY = y;

                double weightSum = 0;
                double weightedX = 0;
                double weightedY = 0;
                double intensitySum = 0;

                while (queue.Count > 0)
                {
                    Vector2Int p = queue.Dequeue();

                    int px = p.x;
                    int py = p.y;

                    int pixelIndex = py * width + px;

                    float weight = intensity[pixelIndex];

                    area++;

                    minX = Math.Min(minX, px);
                    maxX = Math.Max(maxX, px);
                    minY = Math.Min(minY, py);
                    maxY = Math.Max(maxY, py);

                    weightSum += weight;
                    weightedX += px * weight;
                    weightedY += py * weight;

                    intensitySum += weight;

                    for (int n = 0; n < 8; n++)
                    {
                        int nx = px + dx[n];
                        int ny = py + dy[n];

                        if (nx < 0 || nx >= width ||
                            ny < 0 || ny >= height)
                            continue;

                        int neighbourIndex =
                            ny * width + nx;

                        if (!mask[neighbourIndex] ||
                            visited[neighbourIndex])
                            continue;

                        visited[neighbourIndex] = true;

                        queue.Enqueue(
                            new Vector2Int(nx, ny)
                        );
                    }
                }

                int blobWidth = maxX - minX + 1;
                int blobHeight = maxY - minY + 1;

                float aspect =
                    Mathf.Max(blobWidth, blobHeight) /
                    (float)Mathf.Max(
                        1,
                        Mathf.Min(blobWidth, blobHeight)
                    );

                // Same rejection rules as Python
                if (area < markerMinArea ||
                    area > markerMaxArea ||
                    aspect > markerMaxAspect)
                {
                    continue;
                }

                if (weightSum <= 0)
                    continue;

                int cx = Mathf.RoundToInt(
                    (float)(weightedX / weightSum));

                int cy = Mathf.RoundToInt(
                    (float)(weightedY / weightSum));

                float meanIntensity =
                    (float)(intensitySum / area);

                candidates.Add(new Candidate
                {
                    meanIntensity = meanIntensity,
                    center = new Vector2Int(cx, cy)
                });
            }
        }

        return candidates;
    }
}