namespace Braillic.Tracking.Runtime
{
    /// <summary>Optional provider capability. Validation must not mutate state or acquire resources.
    /// Read the accepted configuration from Context.GeometryConfiguration at Start/Pump.</summary>
    public interface IGeometryTrackingProvider
    {
        void ValidateGeometryConfiguration(ProviderGeometryConfiguration configuration);
    }
}
