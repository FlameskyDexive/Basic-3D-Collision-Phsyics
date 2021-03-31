using UnityEngine;
using Unity.Mathematics;

public enum CollisionType
{
    SphereSphere , SphereCapsule , SphereBox , CapsuleLockCapsuleLock , CaspuleCapsule , BoxBox , SphereTriangle
}

public class CollisionTester : MonoBehaviour
{
    public CollisionType collisionType;
    public bool resolveCollisions;
    public bool drawMarkers;

    public float3 posA;
    public float3 posB;

    public float3 rotA;
    public float3 rotB;

    public float3 halfSizeA;
    public float3 halfSizeB;

    private GameObject shapeA;
    private GameObject shapeB;
    private GameObject closestPointMarker;
    private Color normalColor = Color.black;
    private Color collisionColor = Color.red;

    private CollisionType lastCollisionType;
    private bool collisionFlag = false;

    private void Start()
    {
        SetupScene();
    }
    private void Update()
    {
        collisionFlag = false;

        if ( collisionType != lastCollisionType )
        {
            lastCollisionType = collisionType;
            SetupScene();
        }

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            HandleSpheres();
            break;
            case CollisionType.SphereCapsule:
            HandleSphereCapsule();
            break;
            case CollisionType.SphereBox:
            HandleSphereBox();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            HandleCapsulesLock();
            break;
            case CollisionType.CaspuleCapsule:
            HandleCapsuleCapsule();
            break;
            case CollisionType.BoxBox:
            HandleBoxBox3D();
            break;
            case CollisionType.SphereTriangle:
            HandleSphereTriangle();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }

        if ( collisionFlag )
        {
            shapeA.GetComponent<MeshRenderer>().material.color = collisionColor;
            shapeB.GetComponent<MeshRenderer>().material.color = collisionColor;
        }
        else
        {
            shapeA.GetComponent<MeshRenderer>().material.color = normalColor;
            shapeB.GetComponent<MeshRenderer>().material.color = normalColor;
        }
    }

    private void SetupScene()
    {
        ResetTransform();

        if ( shapeA )
            Destroy( shapeA );
        if ( shapeB )
            Destroy( shapeB );

        switch ( collisionType )
        {
            case CollisionType.SphereSphere:
            SetupSpheres();
            break;
            case CollisionType.SphereCapsule:
            SetupSphereCapsule();
            break;
            case CollisionType.SphereBox:
            SetupSphereBox();
            break;
            case CollisionType.CapsuleLockCapsuleLock:
            SetupLockedCapsule();
            break;
            case CollisionType.CaspuleCapsule:
            SetupCapsuleCapsule();
            break;
            case CollisionType.BoxBox:
            SetupBoxBox();
            break;
            case CollisionType.SphereTriangle:
            SetupPointTriangle();
            break;
            default:
            Debug.Log( "No CollisionType Selected" );
            break;
        }

        if ( shapeA )
        {
            Material material = new Material( Shader.Find( "Standard" ) );
            material.color = normalColor;
            shapeA.GetComponent<MeshRenderer>().material = material;
        }
        if ( shapeB )
        {
            Material material = new Material( Shader.Find( "Standard" ) );
            material.color = normalColor;
            shapeB.GetComponent<MeshRenderer>().material = material;
        }
    }
    private void SetupSpheres()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Sphere );
    }
    private void SetupLockedCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupCapsuleCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Capsule );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupBoxBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Cube );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void SetupSphereCapsule()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Capsule );
    }
    private void SetupSphereBox()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = GameObject.CreatePrimitive( PrimitiveType.Cube );
    }
    private void SetupPointTriangle()
    {
        shapeA = GameObject.CreatePrimitive( PrimitiveType.Sphere );
        shapeB = new GameObject();
        Mesh mesh = new Mesh();
        Vector3[] vertices = new Vector3[ 3 ];
        vertices[ 0 ] = posB + new float3( -halfSizeB.x , -halfSizeB.y , 0 );
        vertices[ 1 ] = posB + new float3( halfSizeB.x , -halfSizeB.y , 0 );
        vertices[ 2 ] = posB + new float3( 0 , halfSizeB.y , 0 );
        int[] triangles = new int[ 3 ];
        triangles[ 0 ] = 0;
        triangles[ 1 ] = 1;
        triangles[ 2 ] = 2;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        shapeB.AddComponent<MeshFilter>();
        shapeB.AddComponent<MeshRenderer>();
        shapeB.transform.position = posB;
        shapeB.GetComponent<MeshFilter>().mesh = mesh;
        Material mat = new Material( Shader.Find( "Standard" ) );
        shapeB.GetComponent<MeshRenderer>().material = mat;
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x , rotB.y + DegreesToRadians( 180 ) , rotB.z );
    }
    private void ResetTransform()
    {
        posA = float3.zero;
        posB = new float3( 5 , 0 , 0 );

        rotA = float3.zero;
        rotB = float3.zero;

        halfSizeA = new float3( 1 , 1 , 1 );
        halfSizeB = new float3( 1 , 1 , 1 );
    }

    // SPHERE SPHERE
    private void HandleSpheres()
    {
        CollisionSphere( ref posA , ref posB , halfSizeA.x , halfSizeB.x );
        DrawSpheres();
    }
    private void CollisionSphere( ref float3 _posA , ref float3 _posB , float radiusA , float radiusB )
    {
        float distance = math.distance( _posA , _posB );
        float sumRadii = radiusA + radiusB;
        bool collision = distance <= sumRadii;

        if ( collision )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                float3 displacement = CalculateSphereCollisionDisplacement( _posA , _posB , radiusA , radiusB , distance );

                _posA -= displacement;
                _posB += displacement;
            }
        }
    }

    // SPHERE CAPSULE
    private void HandleSphereCapsule()
    {
        CollisionSphereCapsule( ref posA , halfSizeA.x , ref posB , rotB , halfSizeB.y * 2 , halfSizeB.x );
        DrawSphereCapsule();
    }
    private void CollisionSphereCapsule( ref float3 spherePos , float sphereRadius , ref float3 capsulePos , float3 capsuleRotation , float capsuleLength , float capsuleRadius )
    {
        CapsulePoints capsulePoints = GetCapsuleCollider( capsulePos , capsuleRotation , capsuleLength );
        CapsulePoints capsuleSpheres = GetCapsuleSpheres( capsulePoints.tipPoint , capsulePoints.basePoint , capsuleRadius );

        float3 closestPoint = ClosestPointOnLineSegement( capsuleSpheres.basePoint , capsuleSpheres.tipPoint , spherePos );

        float distance = math.distance( spherePos , closestPoint );
        float penetrationDepth = sphereRadius + capsuleRadius - distance;

        if ( penetrationDepth > 0 )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                CollisionResolveCapsuleCapsule( ref spherePos , ref capsulePos , spherePos , closestPoint , sphereRadius , capsuleRadius , distance );
            }
        }
    }

    // SPHERE BOX
    private void HandleSphereBox()
    {
        float3[] verts = GetRotatedVerticesOfBox( posB , halfSizeB , rotB );
        float3[] normals = GetNormalsOfOBB( verts );

        CollisionSphereBox( ref posA , halfSizeA.x , ref posB , halfSizeB , normals );
        DrawSphereBox();
    }
    private void CollisionSphereBox( ref float3 spherePos , float sphereRadius , ref float3 boxPos , float3 halfSize , float3[] normals )
    {
        float[] extents = new float[] { halfSize.x , halfSize.y , halfSize.z };

        float3 closestPoint = ClosestPointOnOBBToPoint( spherePos , boxPos , normals , extents );
        float distance = math.distance( closestPoint , spherePos );
        bool collision = distance < sphereRadius;

        if ( collision )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                float overlap = sphereRadius - distance;
                float3 collisionDirection = math.normalize( spherePos - boxPos );

                spherePos += collisionDirection * ( overlap / 2 );
                boxPos -= collisionDirection * ( overlap / 2 );
            }
        }
    }

    // CAPSULELOCK CAPSULELOCK
    private void HandleCapsulesLock()
    {
        CollisionCapsuleLock( ref posA , ref posB , halfSizeA.x , halfSizeB.x , halfSizeA.y , halfSizeB.y );
        DrawCapsuleLock();
    }
    private void CollisionCapsuleLock( ref float3 _posA , ref float3 _posB , float radiusA , float radiusB , float heightA, float heightB )
    {
        if ( _posA.y + heightA >= _posB.y - heightB )
        {
            CollisionSphere( ref _posA , ref _posB , radiusA , radiusB );
        }
    }

    // CAPSULE CAPSULE
    private struct CapsulePoints
    {
        public float3 tipPoint;
        public float3 basePoint;
    }
    private void HandleCapsuleCapsule()
    {
        CollisionCapsuleCapsule( ref posA , ref posB , rotA , rotB , halfSizeA.y , halfSizeB.y , halfSizeA.x , halfSizeB.x );
        DrawCapsules();
    }
    private void CollisionCapsuleCapsule( ref float3 _posA , ref float3 _posB , float3 _rotA , float3 _rotB , float lengthA , float lengthB , float radiusA , float radiusB )
    {
        CapsulePoints pointsA = GetCapsuleCollider( _posA , _rotA , lengthA * 2 );
        CapsulePoints pointsB = GetCapsuleCollider( _posB , _rotB , lengthB * 2 );

        CapsulePoints spheresA = GetCapsuleSpheres( pointsA.tipPoint , pointsA.basePoint , radiusA );
        CapsulePoints spheresB = GetCapsuleSpheres( pointsB.tipPoint , pointsB.basePoint , radiusB );

        float3 closestPointA = float3.zero;
        float3 closestPointB = float3.zero;
        float distance = 0;

        bool intersects = CollisionTestCapsuleCapsule( spheresA , spheresB , ref closestPointA , ref closestPointB , ref distance );

        if ( intersects )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                CollisionResolveCapsuleCapsule( ref _posA , ref _posB , closestPointA , closestPointB , radiusA , radiusB , distance );
            }
        }
    }
    private bool CollisionTestCapsuleCapsule( CapsulePoints spheresA , CapsulePoints spheresB , ref float3 closestPointA , ref float3 closestPointB , ref float distance )
    {
        float3 v0 = spheresB.basePoint - spheresA.basePoint;
        float3 v1 = spheresB.tipPoint - spheresA.basePoint;
        float3 v2 = spheresB.basePoint - spheresA.tipPoint;
        float3 v3 = spheresA.tipPoint - spheresA.tipPoint;

        float d0 = math.dot( v0 , v0 );
        float d1 = math.dot( v1 , v1 );
        float d2 = math.dot( v2 , v2 );
        float d3 = math.dot( v3 , v3 );

        closestPointA = math.select( spheresA.basePoint , spheresA.tipPoint , d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1 );
        closestPointB = ClosestPointOnLineSegement( spheresB.basePoint , spheresB.tipPoint , closestPointA );
        closestPointA = ClosestPointOnLineSegement( spheresA.basePoint , spheresA.tipPoint , closestPointB );

        distance = math.distance( closestPointA , closestPointB );
        float penetrationDepth = halfSizeA.x + halfSizeB.x - distance;

        return penetrationDepth > 0;
    }
    private void CollisionResolveCapsuleCapsule( ref float3 originA , ref float3 originB , float3 pointA , float3 pointB , float radiusA , float radiusB , float distance )
    {
        float3 displacement = CalculateSphereCollisionDisplacement( pointA , pointB , radiusA , radiusB , distance );

        originA -= displacement;
        originB += displacement;
    }
    private CapsulePoints GetCapsuleCollider( float3 position , float3 rotation , float length )
    {
        float3 forward = math.forward( quaternion.EulerXYZ( rotation ) );
        float3 axisLine = forward * length;
        float3 tipPoint = position + axisLine;
        float3 basePoint = position - axisLine;

        return new CapsulePoints
        {
            tipPoint = tipPoint ,
            basePoint = basePoint
        };
    }
    private CapsulePoints GetCapsuleSpheres( float3 tipPoint , float3 basePoint , float radius )
    {
        float3 normal = math.normalize( tipPoint -basePoint );
        float3 lineEndOffset = normal * radius;
        float3 baseSphere = basePoint + lineEndOffset;
        float3 tipSphere = tipPoint - lineEndOffset;

        return new CapsulePoints
        {
            basePoint = baseSphere ,
            tipPoint = tipSphere
        };
    }

    // BOX BOX
    private void HandleBoxBox3D()
    {
        float3[] vertsA = GetRotatedVerticesOfBox( posA , halfSizeA , rotA );
        float3[] vertsB = GetRotatedVerticesOfBox( posB , halfSizeB , rotB );

        float3[] axesA = GetNormalsOfOBB( vertsA );
        float3[] axesB = GetNormalsOfOBB( vertsB );

        float3[] allAxes = new float3[ 15 ];
        allAxes[ 0 ] = axesA[ 0 ];
        allAxes[ 1 ] = axesA[ 1 ];
        allAxes[ 2 ] = axesA[ 2 ];
        allAxes[ 3 ] = axesB[ 0 ];
        allAxes[ 4 ] = axesB[ 1 ];
        allAxes[ 5 ] = axesB[ 2 ];
        allAxes[ 6 ] = math.cross( axesA[ 0 ] , axesB[ 0 ] );
        allAxes[ 7 ] = math.cross( axesA[ 0 ] , axesB[ 1 ] );
        allAxes[ 8 ] = math.cross( axesA[ 0 ] , axesB[ 2 ] );
        allAxes[ 9 ] = math.cross( axesA[ 1 ] , axesB[ 0 ] );
        allAxes[ 10 ] = math.cross( axesA[ 1 ] , axesB[ 1 ] );
        allAxes[ 11 ] = math.cross( axesA[ 1 ] , axesB[ 2 ] );
        allAxes[ 12 ] = math.cross( axesA[ 2 ] , axesB[ 0 ] );
        allAxes[ 13 ] = math.cross( axesA[ 2 ] , axesB[ 1 ] );
        allAxes[ 14 ] = math.cross( axesA[ 2 ] , axesB[ 2 ] );

        float minOverlap = float.PositiveInfinity;
        float3 minOverlapAxis = float3.zero;
        bool collision = CollisionTestBoxBox( allAxes , vertsA , vertsB , ref minOverlap , ref minOverlapAxis );

        if ( collision )
        {
            collisionFlag = true;

            if ( resolveCollisions )
            {
                CollisionResolveBoxBox( ref posA , ref posB , minOverlap , minOverlapAxis );
            }
        }

        DrawBoxes();
    }
    private bool CollisionTestBoxBox( float3[] allAxes , float3[] vertsA , float3[] vertsB , ref float minOverlap , ref float3 minOverlapAxis )
    {
        for ( int i = 0; i < allAxes.Length; i++ )
        {
            float projMinA = float.MaxValue;
            float projMinB = float.MaxValue;
            float projMaxA = float.MinValue;
            float projMaxB = float.MinValue;

            float3 axis = allAxes[ i ];

            if ( axis.x == 0 && axis.y == 0 && axis.z == 0 )
                continue;

            for ( int j = 0; j < vertsB.Length; j++ )
            {
                float p = math.dot( vertsB[ j ] , axis );

                projMinB = math.min( p , projMinB );
                projMaxB = math.max( p , projMaxB );
            }
            for ( int j = 0; j < vertsA.Length; j++ )
            {
                float p = math.dot( vertsA[ j ] , axis );

                projMinA = math.min( p , projMinA );
                projMaxA = math.max( p , projMaxA );
            }

            float overlap = GetLineOverLap( projMinA , projMinB , projMaxA , projMaxB );

            if ( overlap <= 0 )
            {
                return false;
            }
            else if ( overlap < minOverlap )
            {
                minOverlap = overlap;
                minOverlapAxis = axis;
            }
        }

        return true;
    }
    private void CollisionResolveBoxBox( ref float3 originA , ref float3 originB , float minOverlap , float3 minOverlapAxis )
    {
        float3 dir = originB - originA;
        float dot = math.dot( minOverlapAxis , dir );

        minOverlapAxis *= math.select( 1 , -1 , dot <= 0 );
        minOverlapAxis = math.normalize( minOverlapAxis );

        float3 displacementA = -( minOverlapAxis * minOverlap ) / 2;
        float3 displacementB = ( minOverlapAxis * minOverlap ) / 2;

        originA += displacementA;
        originB += displacementB;
    }

    // POINT TRIANGLE
    private void HandleSphereTriangle()
    {
        float3[] triangleVerts = GetVerticesOfTriangle( posB , halfSizeB , rotB );
        float3 closestPoint = float3.zero;
        bool inTriangle = CollisionTestTriangle( ref posA , halfSizeA.x , posB , triangleVerts , ref closestPoint );

        if ( inTriangle )
        {
            float distance = math.distance( posA , closestPoint );

            if ( distance < halfSizeA.x )
            {
                collisionFlag = true;

                if ( resolveCollisions )
                {
                    float3 direction = math.normalize( posA - closestPoint );
                    float displacement = distance - halfSizeA.x;

                    posA += direction * displacement;
                }
            }
        }

        DrawSphereTriangle();
    }
    private bool CollisionTestTriangle( ref float3 spherePos , float radius , float3 triangleCenter , float3[] triangleVerts , ref float3 closestPoint )
    {
        closestPoint = ClosestPointTriangle( spherePos , triangleVerts[ 0 ] , triangleVerts[ 1 ] , triangleVerts[ 2 ] );
        int inTriangle = PointInsideTriangle3D( closestPoint , triangleVerts[ 0 ] , triangleVerts[ 1 ] , triangleVerts[ 2 ] );

        return inTriangle > 0;
    }
    private float3 ClosestPointTriangle( float3 p , float3 a , float3 b , float3 c )
    {
        float3 normal = math.cross( ( c - a ) , ( b - a ) );
        float3 closestPoint = p - ( math.dot( normal , ( p - a ) ) / ( math.dot( normal , normal ) ) * normal);
        return closestPoint;
    }

    // HELPERS
    private float GetLineOverLap( float min1 , float min2 , float max1 , float max2 )
    {
        float _min1 = math.min( max1 , max2 );
        float _max1 = math.max( min1 , min2 );
        float diff = _min1 - _max1;

        return math.max( 0 , diff );
    }
    private float DegreesToRadians( float degrees )
    {
        return degrees * ( math.PI / 180 );
    }
    private float3[] GetIdentityVerticesOfBox( float3 origin , float3 halfSize )
    {
        // -z, bl,br,tr,tl +z, bl,br,tr,tl
        float3[] verts = new float3[ 8 ];

        verts[ 0 ] = ( origin + new float3( -halfSize.x , -halfSize.y , -halfSize.z ) );
        verts[ 1 ] = ( origin + new float3( +halfSize.x , -halfSize.y , -halfSize.z ) );
        verts[ 2 ] = ( origin + new float3( +halfSize.x , +halfSize.y , -halfSize.z ) );
        verts[ 3 ] = ( origin + new float3( -halfSize.x , +halfSize.y , -halfSize.z ) );

        verts[ 4 ] = ( origin + new float3( -halfSize.x , -halfSize.y , +halfSize.z ) );
        verts[ 5 ] = ( origin + new float3( +halfSize.x , -halfSize.y , +halfSize.z ) );
        verts[ 6 ] = ( origin + new float3( +halfSize.x , +halfSize.y , +halfSize.z ) );
        verts[ 7 ] = ( origin + new float3( -halfSize.x , +halfSize.y , +halfSize.z ) );

        return verts;
    }
    private float3[] GetRotatedVerticesOfBox( float3 origin , float3 halfSize , float3 rotation )
    {
        float3[] verts = GetIdentityVerticesOfBox( origin , halfSize );

        for ( int i = 0; i < verts.Length; i++ )
            verts[ i ] = RotatePoint3D( verts[ i ] , origin , rotation.y , rotation.z , rotation.x );

        return verts;
    }
    private float3[] GetNormalsOfOBB( float3[] vertices )
    {
        // x,y,z
        float3[] axes = new float3[ 3 ];
        axes[ 0 ] = math.normalize( ( vertices[ 1 ] - vertices[ 0 ] ) );
        axes[ 1 ] = math.normalize( ( vertices[ 3 ] - vertices[ 0 ] ) );
        axes[ 2 ] = math.normalize( ( vertices[ 4 ] - vertices[ 0 ] ) );

        return axes;
    }
    private float3 CalculateSphereCollisionDisplacement( float3 pointA , float3 pointB , float radiusA , float radiusB , float distance )
    {
        float overlap = 0.5f * ( distance - radiusA - radiusB );
        float3 displacement = ( overlap * ( pointA - pointB ) ) / ( distance + 0.0001f );

        return displacement;
    }
    private float3 ClosestPointOnLineSegement( float3 a , float3 b , float3 point )
    {
        float3 ab = b - a;
        float t = math.dot( point - a , ab ) / math.dot( ab , ab );
        return a + math.saturate( t ) * ab;
    }
    private float3 RotatePoint3D( float3 point , float3 origin , float pitch , float yaw , float roll )
    {
        float cosa = math.cos( yaw );
        float sina = math.sin( yaw );

        float cosb = math.cos( pitch );
        float sinb = math.sin( pitch );

        float cosc = math.cos( roll );
        float sinc = math.sin( roll );

        float xxA = cosa * cosb;
        float xyA = cosa * sinb * sinc - sina * cosc;
        float xzA = cosa * sinb * cosc + sina * sinc;

        float yxA = sina * cosb;
        float yyA = sina * sinb * sinc + cosa * cosc;
        float yzA = sina * sinb * cosc - cosa * sinc;

        float zxA = -sinb;
        float zyA = cosb * sinc;
        float zzA = cosb * cosc;

        float px = point.x;
        float py = point.y;
        float pz = point.z;

        float pxR = origin.x + xxA * ( px - origin.x ) + xyA * ( py - origin.y ) + xzA * ( pz - origin.z );
        float pyR = origin.y + yxA * ( px - origin.x ) + yyA * ( py - origin.y ) + yzA * ( pz - origin.z );
        float pzR = origin.z + zxA * ( px - origin.x ) + zyA * ( py - origin.y ) + zzA * ( pz - origin.z );

        return new float3( pxR , pyR , pzR );
    }
    private float3 ClosestPointOnOBBToPoint( float3 p , float3 c , float3[] obbNormals , float[] extents )
    {
        // c = center point, u = directional unit vector
        // all points contained by obb b can be written as
        // s = c + au0 + bu1 + cu2

        // point p in orld space can be represented as point q in obb space as
        // q = c + xu0 + yu1 + zu2
        // x = (p-c) dot u0 , y = (p-c) dot u1 , z = (p-c) dot u2

        float3 d = p - c;
        float3 q = c;

        for ( int i = 0; i < 3; i++ )
        {
            float dist = math.dot( d , obbNormals[ i ] );

            dist = math.max( dist , -extents[ i ] );
            dist = math.min( dist , extents[ i ] );

            q += dist * obbNormals[ i ];
        }

        return q;
    }
    private float3[] GetVerticesOfTriangle( float3 position , float3 halfSize , float3 rotation )
    {
        float3[] vertices = new float3[ 3 ];
        vertices[ 0 ] = posB + new float3( -halfSize.x , -halfSize.y , 0 );
        vertices[ 1 ] = posB + new float3( halfSize.x , -halfSize.y , 0 );
        vertices[ 2 ] = posB + new float3( 0 , halfSize.y , 0 );

        for ( int i = 0; i < vertices.Length; i++ )
        {
            vertices[ i ] = RotatePoint3D( vertices[ i ] , position , rotation.y , rotation.z , rotation.x );
        }

        return vertices;
    }
    private int PointInsideTriangle3D( double3 p , double3 a , double3 b , double3 c )
    {
        int isInside = 1;

        a -= p;
        b -= p;
        c -= p;

        double3 u = math.cross( b , c );
        double3 v = math.cross( c , a );
        double3 w = math.cross( a , b );

        isInside = math.select( 1 , 0 , math.dot( u , v ) <= 0 );
        isInside = math.select( 1 , 0 , math.dot( u , w ) <= 0 );

        return isInside;
    }
    private bool PointInsideTriangle2D( double x1 , double y1 , double x2 , double y2 , double x3 , double y3 , double x , double y )
    {
        double A = AreaOfTriangle( x1 , y1 , x2 , y2 , x3 , y3 ); // Calculate area of triangle ABC
        double A1 = AreaOfTriangle( x , y , x2 , y2 , x3 , y3 ); // Calculate area of triangle PBC
        double A2 = AreaOfTriangle( x1 , y1 , x , y , x3 , y3 ); // Calculate area of triangle PAC
        double A3 = AreaOfTriangle( x1 , y1 , x2 , y2 , x , y ); // Calculate area of triangle PAB
        return ( A == A1 + A2 + A3 ); // Check if sum of A1, A2 and A3 is same as A
    }
    private double AreaOfTriangle( double x1 , double z1 , double x2 , double z2 , double x3 , double z3 )
    {
        return math.abs( ( x1 * ( z2 - z3 ) + x2 * ( z3 - z1 ) + x3 * ( z1 - z2 ) ) / 2.0f );
    }

    // DRAW FUNCTIONS
    private void DrawSpheres()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawSphereCapsule()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = DegreesToRadians( 90 );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.x * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawSphereBox()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawCapsuleLock()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawCapsules()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        float xRotation = DegreesToRadians( 90 );
        shapeA.transform.rotation = quaternion.EulerXYZ( rotA.x + xRotation , rotA.y , rotA.z );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB.x + xRotation , rotB.y , rotB.z );

        shapeA.transform.localScale = new float3( halfSizeA.x * 2 , halfSizeA.y * 2 , halfSizeA.x * 2 );
        shapeB.transform.localScale = new float3( halfSizeB.x * 2 , halfSizeB.y * 2 , halfSizeB.x * 2 );
    }
    private void DrawBoxes()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.rotation = quaternion.EulerXYZ( rotA );
        shapeB.transform.rotation = quaternion.EulerXYZ( rotB );

        shapeA.transform.localScale = halfSizeA * 2;
        shapeB.transform.localScale = halfSizeB * 2;
    }
    private void DrawSphereTriangle()
    {
        shapeA.transform.position = posA;
        shapeB.transform.position = posB;

        shapeA.transform.localScale = halfSizeA * 2;

        float3[] verts = GetVerticesOfTriangle( posB , halfSizeB , rotB );
        Vector3[] vertsVector = new Vector3[ 3 ];
        for ( int i = 0; i < verts.Length; i++ )
        {
            vertsVector[ i ] = verts[ i ];
        }

        shapeB.GetComponent<MeshFilter>().mesh.vertices = vertsVector;
        shapeB.GetComponent<MeshFilter>().mesh.RecalculateNormals();
    }
}
