import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Label } from "@/components/ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Telescope, Loader2 } from "lucide-react";
import { toast } from "sonner";
import { useExplorerMode } from "@/hooks/useFeatureFlag";
import { ExplorerResults } from "@/components/ExplorerResults";
import { ResearcherOnly } from "@/components/ResearcherOnly";
import { BaselineComparison } from "@/components/BaselineComparison";
import { sampleCandidates, Candidate } from "@/data/sampleCandidates";

const Detect = () => {
  const [isProcessing, setIsProcessing] = useState(false);
  const [results, setResults] = useState<Candidate[]>([]);
  const [selectedCandidate, setSelectedCandidate] = useState<Candidate | null>(null);
  const [showComparison, setShowComparison] = useState(false);
  const isExplorerMode = useExplorerMode();

  const handleRunDemo = (candidateIndex?: number) => {
    setIsProcessing(true);
    setShowComparison(false);
    toast.info(isExplorerMode 
      ? "Starting discovery process..." 
      : "Starting analysis on pre-loaded sample data..."
    );
    
    setTimeout(() => {
      setIsProcessing(false);
      setResults(sampleCandidates);
      
      if (candidateIndex !== undefined) {
        setSelectedCandidate(sampleCandidates[candidateIndex]);
      }
      
      const genuineCount = sampleCandidates.filter(c => !c.isFalsePositive).length;
      
      toast.success(isExplorerMode
        ? `Discovery complete! Found ${genuineCount} potential exoplanet${genuineCount !== 1 ? 's' : ''}!`
        : `Analysis complete! Found ${sampleCandidates.length} candidates (${genuineCount} genuine, ${sampleCandidates.filter(c => c.isFalsePositive).length} false positive).`
      );
    }, 2000);
  };

  const handleViewComparison = (candidate: Candidate) => {
    setSelectedCandidate(candidate);
    setShowComparison(true);
  };

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8">
        <h1 className="text-3xl font-bold mb-2">
          {isExplorerMode ? "Discover Exoplanets" : "Transit Detection"}
        </h1>
        <p className="text-muted-foreground mb-4">
          {isExplorerMode 
            ? "Start your journey of discovery using our pre-loaded sample data"
            : "Run analysis on pre-loaded sample data or upload your own light curves"
          }
        </p>
      </div>

      <div className="grid lg:grid-cols-3 gap-6">
        {/* Left Panel - Input */}
        <div className="lg:col-span-1 space-y-6">
          <Card>
            <CardHeader>
              <CardTitle>Data Source</CardTitle>
              <CardDescription>
                {isExplorerMode 
                  ? "Start with our curated sample datasets"
                  : "Choose where to load light curve data"
                }
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                <div>
                  <Label>Sample Dataset</Label>
                  <Select defaultValue="kepler">
                    <SelectTrigger>
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="kepler">Kepler Sample Data</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                <ResearcherOnly>
                  <div className="pt-4 border-t">
                    <Label>Advanced Options</Label>
                    <div className="grid grid-cols-2 gap-4 mt-2">
                      <Button variant="outline" size="sm">
                        Custom Parameters
                      </Button>
                      <Button variant="outline" size="sm">
                        Upload Data
                      </Button>
                    </div>
                  </div>
                </ResearcherOnly>

                <Button 
                  variant="default" 
                  className="w-full"
                  onClick={() => handleRunDemo()}
                  disabled={isProcessing}
                >
                  {isProcessing ? (
                    <Loader2 className="h-4 w-4 animate-spin" />
                  ) : (
                    <Telescope className="h-4 w-4 mr-2" />
                  )}
                  {isExplorerMode ? "Start Discovery" : "Run Analysis"}
                </Button>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Main Content Area */}
        <div className="lg:col-span-2">
          {results.length > 0 && (
            <>
              {isExplorerMode ? (
                <ExplorerResults 
                  results={results}
                  onViewDetails={handleViewComparison}
                />
              ) : (
                <div className="space-y-6">
                  {results.map((candidate) => (
                    <Card key={candidate.id}>
                      <CardHeader>
                        <div className="flex items-center justify-between">
                          <CardTitle>{candidate.name}</CardTitle>
                          <Button
                            variant="outline"
                            size="sm"
                            onClick={() => handleViewComparison(candidate)}
                          >
                            View Details
                          </Button>
                        </div>
                        <CardDescription>
                          {candidate.description}
                        </CardDescription>
                      </CardHeader>
                    </Card>
                  ))}
                </div>
              )}

              {showComparison && selectedCandidate && (
                <div className="mt-6">
                  <BaselineComparison candidate={selectedCandidate} />
                </div>
              )}
            </>
          )}
        </div>
      </div>
    </div>
  );
};

export default Detect;