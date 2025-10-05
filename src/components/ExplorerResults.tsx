import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { CheckCircle, AlertCircle } from "lucide-react";
import type { Candidate } from "@/data/sampleCandidates";

interface ExplorerResultsProps {
  results: Candidate[];
  onViewDetails: (candidate: Candidate) => void;
}

export const ExplorerResults = ({ results, onViewDetails }: ExplorerResultsProps) => {
  const genuineCandidates = results.filter(c => !c.isFalsePositive);
  
  return (
    <div className="space-y-6">
      <div className="bg-muted/50 p-4 rounded-lg">
        <h3 className="text-lg font-medium mb-2">Discovery Summary</h3>
        <p className="text-muted-foreground">
          Found {genuineCandidates.length} potential exoplanet{genuineCandidates.length !== 1 ? "s" : ""}!
        </p>
      </div>

      <div className="grid gap-4">
        {genuineCandidates.map((candidate) => (
          <Card key={candidate.id}>
            <CardHeader>
              <div className="flex items-center justify-between">
                <CardTitle className="text-lg">
                  {candidate.name}
                </CardTitle>
                <Badge variant={candidate.confidence > 0.8 ? "default" : "secondary"}>
                  {candidate.confidence > 0.8 ? "High" : "Medium"} Confidence
                </Badge>
              </div>
            </CardHeader>
            <CardContent>
              <div className="space-y-2">
                <p className="text-sm text-muted-foreground">
                  {candidate.simplifiedDescription || candidate.description}
                </p>
                <Button
                  variant="outline"
                  onClick={() => onViewDetails(candidate)}
                  className="w-full"
                >
                  View Details
                </Button>
              </div>
            </CardContent>
          </Card>
        ))}
      </div>
    </div>
  );
};