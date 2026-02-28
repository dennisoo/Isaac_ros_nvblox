using UnityEngine;
using Siccity.GLTFUtility;
using System.Collections;
using System.IO;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class LoadGLB : MonoBehaviour
{
    [Header("File Settings")]
    public string meshPath = "\\\\wsl.localhost\\Ubuntu\\home\\dennis\\workspaces\\isaac_ros-dev\\meshes\\semantic_tiago_mesh.glb";
    public float checkInterval = 0.5f;
    
    [Header("Behavior")]
    public bool autoLoadOnStart = true;
    public float startDelay = 2.0f;
    
    private bool isLoading = false;
    
    void Start()
    {
        
        if (autoLoadOnStart)
        {
            Invoke(nameof(StartWatching), startDelay);
        }
    }
    
    public void StartWatching()
    {
        if (!isLoading)
        {
            StartCoroutine(WaitForFileAndLoad());
        }
    }
    
    public void LoadMeshNow()
    {
        if (isLoading)
        {
            return;
        }
        
        if (File.Exists(meshPath))
        {
            LoadMesh(meshPath);
        }
        else
        {
            Debug.LogError($"File not found: {meshPath}");
        }
    }
    
    IEnumerator WaitForFileAndLoad()
    {
        isLoading = true;
        
        Debug.Log($"Waiting for file: {meshPath}");
        
        float timeout = 120f;
        float elapsed = 0f;
        
        // Wait for file to exist
        while (!File.Exists(meshPath) && elapsed < timeout)
        {
            yield return new WaitForSeconds(checkInterval);
            elapsed += checkInterval;
            
            if (elapsed % 10f < checkInterval)
            {
                Debug.Log($"Still waiting... ({elapsed:F0}s)");
            }
        }
        
        if (!File.Exists(meshPath))
        {
            Debug.LogError($"Timeout ({timeout}s)");
            isLoading = false;
            yield break;
        }
        
        Debug.Log("File found! Waiting for write to complete...");
        
        // Wait for file to be unlocked
        int lockChecks = 0;
        while (IsFileLocked(meshPath) && lockChecks < 20)
        {
            yield return new WaitForSeconds(checkInterval);
            lockChecks++;
        }
        
        Debug.Log("Loading mesh...");
        LoadMesh(meshPath);
        
        isLoading = false;
    }
    
    void LoadMesh(string path)
    {
        try
        {
            Debug.Log($"Loading: {path}");
            
            GameObject loadedModel = Importer.LoadFromFile(path);
            
            if (loadedModel == null)
            {
                Debug.LogError("Failed to load GLB");
                return;
            }
            
            MeshFilter sourceMeshFilter = loadedModel.GetComponentInChildren<MeshFilter>();
            
            if (sourceMeshFilter == null)
            {
                Debug.LogError("No mesh in GLB");
                Destroy(loadedModel);
                return;
            }
            
            // Transfer mesh
            GetComponent<MeshFilter>().mesh = sourceMeshFilter.mesh;
            
            // Vertex color shader
            Material mat = new Material(Shader.Find("Particles/Standard Unlit"));
            mat.EnableKeyword("_VERTEXCOLOR_ON");
            mat.SetInt("_ColorMode", 1);
            GetComponent<MeshRenderer>().material = mat;
            
            // ROS → Unity rotation
            transform.localEulerAngles = new Vector3(-90f, 0f, 0f);
            
            Destroy(loadedModel);
            
            Mesh m = GetComponent<MeshFilter>().mesh;
            Debug.Log($"Loaded! Vertices: {m.vertexCount:N0}, Triangles: {m.triangles.Length/3:N0}");
            Debug.Log($"Size: {new FileInfo(path).Length / (1024f * 1024f):F2} MB");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error: {e.Message}");
        }
    }
    
    bool IsFileLocked(string filePath)
    {
        try
        {
            using (FileStream stream = File.Open(filePath, FileMode.Open, FileAccess.Read, FileShare.None))
            {
                stream.Close();
            }
        }
        catch (IOException)
        {
            return true;
        }
        return false;
    }
}