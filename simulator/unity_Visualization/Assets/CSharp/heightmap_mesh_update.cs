using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Assertions;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class heightmap_mesh_update : MonoBehaviour
{

	private Mesh mesh;
    private Texture2D uvmap;
    private Vector3[] vertices;

    // Start is called before the first frame update
    void Start()
    {
    	// Don't want to actually run on start, want to run when we start picking up LCM stream
    	ResetMesh(3, 3);
        
    }

    private void ResetMesh(int width, int height)
    {
        Assert.IsTrue(SystemInfo.SupportsTextureFormat(TextureFormat.RGFloat));
        uvmap = new Texture2D(width, height, TextureFormat.RGFloat, false, true)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Point,
        };
        GetComponent<MeshRenderer>().sharedMaterial.SetTexture("_UVMap", uvmap);

        if (mesh != null)
            mesh.Clear();
        else
            mesh = new Mesh()
            {
                indexFormat = IndexFormat.UInt32,
            };

        vertices = new Vector3[width * height];

        var indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            indices[i] = i;

        mesh.MarkDynamic();
        mesh.vertices = vertices;

        var uvs = new Vector2[width * height];
        Array.Clear(uvs, 0, uvs.Length);
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                uvs[i + j * width].x = i / (float)width;
                uvs[i + j * width].y = j / (float)height;
            }
        }

        mesh.uv = uvs;

        mesh.SetIndices(indices, MeshTopology.Lines, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);

        GetComponent<MeshFilter>().sharedMesh = mesh;
    }

    // Update is called once per frame
    void Update()
    {
    	for (int i = 0; i < 9; i++){
    		vertices[i] = new Vector3((float) i / 4.0f, (float) i / 4.0f, (float) i / 4.0f);
    	}

    	mesh.vertices = vertices;
        mesh.UploadMeshData(false);
        
    }
}
